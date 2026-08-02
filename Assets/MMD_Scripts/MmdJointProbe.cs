using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using UnityEngine;

/// <summary>
/// ジョイントプローブ v2：スカートのジョイントを「横（同一段）／縦（段間）／取付（体⇔スカート）」に
/// 分類し、横は伸びを径方向・接線方向に分解、縦・取付は曲げ角を計測する。
/// freeLateralLinear をONにすると横ジョイントの直線リミットをFreeにするA/Bテストができる。
///
/// 使い方：MmdSpinTestと同じルートにアタッチしてPlay。
/// </summary>
public class MmdJointProbe : MonoBehaviour
{
    [Tooltip("スカート剛体の名前に含まれる文字列")]
    public string nameFilter = "スカート";

    [Header("A/Bテスト")]
    [Tooltip("ONにすると横ジョイントの直線モーションをFreeにする（遊び上限を撤廃）。開き角が跳ねるかの白黒付け用")]
    public bool freeLateralLinear = false;

    [Tooltip("ONにするとスカートのコライダーと体・脚など非スカートコライダーの衝突を全て無効化する。接触が開きを封じているかの白黒付け用。テスト中はMmdCollisionMaskのRestore When SeparatedをOFF推奨（復活処理と喧嘩しないように）")]
    public bool disableSkirtBodyCollision = false;

    [Header("計測")]
    public float logInterval = 1f;
    public float settleSeconds = 3f;

    [Header("★ターンイベント計測")]
    [Tooltip("瞬間回転速度がこの値[°/s]を超えたら「ターンイベント」として追跡を始める")]
    public float turnEventThreshold = 360f;
    [Tooltip("回転速度がしきい値を下回ってからこの秒数はイベント継続とみなす（ホイップの余韻を含めるため）")]
    public float turnEventTailSeconds = 0.6f;

    private enum Kind { Lateral, Vertical, Waist }

    private class Probe
    {
        public Kind kind;
        public int ring = -1; // スカート_{段}_{列} の段番号
        public ConfigurableJoint joint;
        public Rigidbody own;
        public Rigidbody connected;
        public Quaternion initialRelRot;

        // ★到達率計測用（取付ジョイントのみ使用）
        //   ジョイント座標系（X=axis, Y=secondaryAxis, Z=X×Y）を保存し、
        //   バインド姿勢からの相対回転をSwing/Twist分解してリミットと突き合わせる。
        public Quaternion jointFrame = Quaternion.identity;
        public float gMaxConeUtil = -1f;   // 全期間の円錐到達率max（-1=計測対象外/制限なし）
        public float gConeSyAtMax;         // 到達率max時のスイングY成分[deg]
        public float gConeSzAtMax;         // 到達率max時のスイングZ成分[deg]
        public float gMaxTwistUtil = -1f;  // 全期間のツイスト到達率max
        public float gTwistAtMax;          // ツイスト到達率max時のツイスト角[deg]
        public float gMaxTotalBend;        // 全期間の総曲げ角max[deg]
        public float intervalMaxConeUtil = -1f; // 定期ログ用（区間ごとにリセット）
        public string limitNote = "";      // Free/Locked軸の注記
    }

    private readonly List<Probe> _probes = new List<Probe>();
    private float _elapsed;
    private float _nextLog;
    private static readonly Regex RingRegex = new Regex(@"_(\d+)_(\d+)");

    // ★剛体位置ベースの「本当の半径」計測用。
    //   横ジョイントの伸び(径/接線分解)は「隣どうしの継ぎ目の開き」であって
    //   「パネル自体が中心からどれだけ離れたか」ではないため、扇状に開く動きだと
    //   接線ばかりに出て径に出にくい可能性がある。これを切り分けるため、
    //   各スカート剛体の水平位置そのものから中心までの距離を直接測る。
    private class RadiusProbe
    {
        public Rigidbody rb;
        public int ring;
        public float baselineRadius; // settle直後（スピン開始前）を基準値とする
        public float baselineRelY;   // ★下半身を基準にした相対Y（settle直後）。裾の「持ち上がり」計測用
    }
    private readonly List<RadiusProbe> _radiusProbes = new List<RadiusProbe>();
    private bool _radiusBaselineCaptured = false;

    // ★総合取付曲げ計測用：下半身→実パネルの直接比較で、内部ジョイント構造に依存しない。
    private class TotalBendProbe
    {
        public Rigidbody rb;
        public Quaternion initialRelRot;
    }
    private readonly List<TotalBendProbe> _totalBendProbes = new List<TotalBendProbe>();

    // ★上書き検出用：FixedUpdate直後(物理計算の直後)の代表ボーン(段2の1枚)の
    //   localRotation/localPositionを覚えておき、LateUpdate(描画直前)で
    //   食い違いがないか比較する。AnimationやIKが物理結果を毎フレーム
    //   書き戻していないかを直接切り分けるための仕掛け。
    private Transform _overwriteWatchBone;
    private Quaternion _afterFixedUpdateRot;
    private Vector3 _afterFixedUpdatePos;
    private bool _watchBoneCaptured = false;
    private float _maxOverwriteDeg = 0f;
    private float _maxOverwritePosMm = 0f;
    private int _overwriteEventCount = 0;
    private float _nextOverwriteLog = 0f;

    // 取付アンカー公転の実測用
    private Probe _waistRef;
    private float _prevYaw;
    private Vector3 _prevAnchor;
    private float _prevTime;
    private float _yawAccum;        // 毎ステップの回転量の積算 [deg]
    private float _anchorPathAccum; // 毎ステップのアンカー移動距離の積算 [m]

    // ★ターンイベント計測：1秒平均では0.25秒の720°/sスパイクが180°/sに潰れて
    //   見えるため、瞬間回転速度(1ステップの回転量/Δt)でイベントを検出し、
    //   その窓内でのスカート応答(持ち上がり・半径・取付曲げ)の最大値を記録する。
    private float _intervalPeakYawRate;   // 定期ログ用の区間内ピーク [°/s]
    private bool _inTurnEvent;
    private float _eventStartTime;
    private float _eventLastAboveTime;
    private float _eventPeakRate;
    private float _eventMaxLiftMm;
    private float _eventMaxRadiusMm;
    private float _eventMaxTotalBend;
    private float _eventMaxTiltDeg;       // ★傾き(スイング成分のみ)：ヨー遅れを除いた「見た目の翻り」
    private int _turnEventCount;
    private float _bestEventPeakRate;
    private float _bestEventLiftMm;
    private float _bestEventBend;
    private float _bestEventTiltDeg;
    private readonly StringBuilder _turnEventReport = new StringBuilder();

    private static string PathOf(Transform t)
    {
        var s = t.name;
        while (t.parent != null) { t = t.parent; s = t.parent == null ? s : t.name + "/" + s; }
        return s;
    }

    private void Start()
    {
        int freed = 0;

        foreach (var j in GetComponentsInChildren<ConfigurableJoint>())
        {
            var own = j.GetComponent<Rigidbody>();
            var conn = j.connectedBody;
            if (own == null || conn == null) continue;

            bool ownIs = own.name.Contains(nameFilter);
            bool connIs = conn.name.Contains(nameFilter);
            if (!ownIs && !connIs) continue;

            Kind kind;
            if (ownIs && connIs)
            {
                bool anyLinear = j.xMotion != ConfigurableJointMotion.Locked
                              || j.yMotion != ConfigurableJointMotion.Locked
                              || j.zMotion != ConfigurableJointMotion.Locked;
                kind = anyLinear ? Kind.Lateral : Kind.Vertical;
            }
            else
            {
                kind = Kind.Waist; // 体⇔スカートの取付ジョイント
            }

            var p = new Probe
            {
                kind = kind,
                joint = j,
                own = own,
                connected = conn,
                initialRelRot = Quaternion.Inverse(conn.rotation) * own.rotation,
            };
            var skirtName = ownIs ? own.name : conn.name;
            var m = RingRegex.Match(skirtName);
            if (m.Success) p.ring = int.Parse(m.Groups[1].Value);

            // ★取付ジョイントのみ：ジョイント座標系を確定して保存（到達率計測用）。
            //   Unityのリミットはこの座標系（X=axis, Y=secondaryAxis）で評価されるため、
            //   実測角も同じ座標系に直してから比較する。
            if (kind == Kind.Waist)
            {
                Vector3 x = j.axis.sqrMagnitude > 1e-8f ? j.axis.normalized : Vector3.right;
                Vector3 y = j.secondaryAxis - Vector3.Dot(j.secondaryAxis, x) * x;
                if (y.sqrMagnitude < 1e-8f)
                    y = Vector3.Cross(x, Mathf.Abs(x.y) < 0.9f ? Vector3.up : Vector3.forward);
                y.Normalize();
                Vector3 z = Vector3.Cross(x, y);
                p.jointFrame = Quaternion.LookRotation(z, y); // X=cross(y,z)=axis になる

                var notes = new List<string>();
                if (j.angularYMotion == ConfigurableJointMotion.Free) notes.Add("Y=Free");
                if (j.angularYMotion == ConfigurableJointMotion.Locked) notes.Add("Y=Locked");
                if (j.angularZMotion == ConfigurableJointMotion.Free) notes.Add("Z=Free");
                if (j.angularZMotion == ConfigurableJointMotion.Locked) notes.Add("Z=Locked");
                if (j.angularXMotion == ConfigurableJointMotion.Free) notes.Add("X=Free");
                if (j.angularXMotion == ConfigurableJointMotion.Locked) notes.Add("X=Locked");
                p.limitNote = notes.Count > 0 ? "[" + string.Join(",", notes) + "]" : "";
            }
            _probes.Add(p);

            if (freeLateralLinear && kind == Kind.Lateral)
            {
                j.xMotion = ConfigurableJointMotion.Free;
                j.yMotion = ConfigurableJointMotion.Free;
                j.zMotion = ConfigurableJointMotion.Free;
                freed++;
            }
        }

        int lat = _probes.Count(p => p.kind == Kind.Lateral);
        int ver = _probes.Count(p => p.kind == Kind.Vertical);
        int wai = _probes.Count(p => p.kind == Kind.Waist);

        // ★半径計測用：横／縦ジョイントに登場するスカート剛体を重複なく集める
        //   （own/connectedの両方から拾う。取付側の相手＝下半身などは除外）。
        var seenRb = new HashSet<Rigidbody>();
        foreach (var p in _probes)
        {
            if (p.kind == Kind.Waist) continue;
            foreach (var rb in new[] { p.own, p.connected })
            {
                if (rb == null || seenRb.Contains(rb)) continue;
                if (!rb.name.Contains(nameFilter)) continue;
                var rm = RingRegex.Match(rb.name);
                if (!rm.Success) continue;
                seenRb.Add(rb);
                _radiusProbes.Add(new RadiusProbe { rb = rb, ring = int.Parse(rm.Groups[1].Value) });
            }
        }

        _overwriteWatchBone = _radiusProbes.FirstOrDefault(p => p.ring == 2)?.rb?.transform
                              ?? _radiusProbes.FirstOrDefault()?.rb?.transform;
        if (_overwriteWatchBone != null)
            Debug.Log($"[JointProbe2][上書き監視] 監視対象: {PathOf(_overwriteWatchBone)}");

        // 取付ジョイント（体⇔スカート）の詳細設定ダンプ：角度制限とドライブ（復元ばね）の在り処を暴く
        var dump = new StringBuilder();
        foreach (var p in _probes.Where(x => x.kind == Kind.Waist))
        {
            var j = p.joint;
            dump.AppendLine(
                $"  取付 {p.own.name} ⇔ {p.connected.name} | " +
                $"直線=({j.xMotion}/{j.yMotion}/{j.zMotion}) limit={j.linearLimit.limit:F4}m | " +
                $"角度=({j.angularXMotion}/{j.angularYMotion}/{j.angularZMotion}) " +
                $"xLow={j.lowAngularXLimit.limit:F1}° xHigh={j.highAngularXLimit.limit:F1}° " +
                $"y={j.angularYLimit.limit:F1}° z={j.angularZLimit.limit:F1}° " +
                $"角リミットばね=(x:{j.angularXLimitSpring.spring:F1}/damp{j.angularXLimitSpring.damper:F1} " +
                $"yz:{j.angularYZLimitSpring.spring:F1}/damp{j.angularYZLimitSpring.damper:F1}) | " +
                $"driveMode={j.rotationDriveMode} " +
                $"角Drive x=({j.angularXDrive.positionSpring:F2}/damp{j.angularXDrive.positionDamper:F2}) " +
                $"yz=({j.angularYZDrive.positionSpring:F2}/damp{j.angularYZDrive.positionDamper:F2}) " +
                $"slerp=({j.slerpDrive.positionSpring:F2}/damp{j.slerpDrive.positionDamper:F2})");
        }

        _waistRef = _probes.FirstOrDefault(p => p.kind == Kind.Waist);
        if (_waistRef != null)
        {
            var cb = _waistRef.connected.transform;
            _prevYaw = cb.eulerAngles.y;
            _prevAnchor = cb.TransformPoint(_waistRef.joint.connectedAnchor);
            _prevTime = 0f;
            dump.AppendLine($"  ★基準: {_waistRef.own.name} の相手剛体のフルパス = {PathOf(cb)} (entityId={cb.GetEntityId()})");
            Vector3 rel = _prevAnchor - cb.position; rel.y = 0f;
            dump.AppendLine($"  ★基準: connectedAnchorの水平半径(相手剛体の中心から) = {rel.magnitude * 1000f:F1}mm");

            // ★総合取付曲げ：下半身→実際のスカートパネル(段0)のワールド回転差を直接測る。
            //   Swing/Twist分離のように内部ジョイント本数が変わる実装でも、
            //   中継剛体の分類に依存せず一貫して比較できる指標として追加。
            Quaternion waistRest = cb.rotation;
            foreach (var rp in _radiusProbes)
            {
                if (rp.ring != 0 || rp.rb == null) continue;
                _totalBendProbes.Add(new TotalBendProbe
                {
                    rb = rp.rb,
                    initialRelRot = Quaternion.Inverse(waistRest) * rp.rb.rotation,
                });
            }
        }

        Debug.Log($"[JointProbe2] 横={lat} 縦={ver} 取付={wai} " +
                  (freeLateralLinear ? $"／横{freed}件の直線をFreeにしました(A/Bテスト)" : "／リミットは既定のまま") +
                  $"\n取付ジョイント詳細:\n{dump}");

        if (disableSkirtBodyCollision)
        {
            var all = GetComponentsInChildren<Collider>();
            var skirtCols = all.Where(c => c.name.Contains(nameFilter) || c.attachedRigidbody != null && c.attachedRigidbody.name.Contains(nameFilter)).ToArray();
            var otherCols = all.Except(skirtCols).ToArray();
            int pairs = 0;
            foreach (var s in skirtCols)
                foreach (var o in otherCols)
                {
                    Physics.IgnoreCollision(s, o, true);
                    pairs++;
                }
            Debug.Log($"[JointProbe2] A/B: スカート({skirtCols.Length}個)⇔非スカート({otherCols.Length}個)の衝突 {pairs} 組を無効化しました");
        }
    }

    private void FixedUpdate()
    {
        _elapsed += Time.fixedDeltaTime;

        // ★計測は毎ステップ積算（1秒間隔の差分だと360°/s回転がちょうど1回転して0に見えるエイリアスが起きるため）
        if (_waistRef != null && _waistRef.connected != null)
        {
            var cb = _waistRef.connected.transform;
            float yaw = cb.eulerAngles.y;
            float stepDeg = Mathf.Abs(Mathf.DeltaAngle(_prevYaw, yaw));
            _yawAccum += stepDeg;
            _prevYaw = yaw;
            Vector3 anchor = cb.TransformPoint(_waistRef.joint.connectedAnchor);
            _anchorPathAccum += (anchor - _prevAnchor).magnitude;
            _prevAnchor = anchor;

            // ★瞬間回転速度[°/s]：1秒平均では潰れるターンのスパイクをそのまま捕まえる
            float instRate = stepDeg / Mathf.Max(Time.fixedDeltaTime, 1e-5f);
            if (instRate > _intervalPeakYawRate) _intervalPeakYawRate = instRate;
            TrackTurnEvent(instRate);
        }

        // ★上書き検出：物理ステップ直後の「正しい」値をここで捕まえる。
        //   settle/log間隔のガードより前、毎FixedUpdateで必ず実行する。
        if (_overwriteWatchBone != null)
        {
            _afterFixedUpdateRot = _overwriteWatchBone.localRotation;
            _afterFixedUpdatePos = _overwriteWatchBone.localPosition;
            _watchBoneCaptured = true;
        }

        // ★到達率は毎ステップ追跡（1秒サンプリングだと回転中のピークを取り逃すため）。
        //   settle中の暴れを混ぜないよう、settle後のみ計測する。
        if (_elapsed >= settleSeconds) TrackWaistUtilization();

        if (_elapsed < settleSeconds || _elapsed < _nextLog) return;
        _nextLog = _elapsed + logInterval;
        LogSummary();
    }

    /// <summary>
    /// ★取付ジョイントごとに、バインド姿勢からの相対回転をジョイント座標系で
    /// Swing/Twist分解し、設定リミットに対する到達率を更新する。
    /// 円錐到達率 = √((sy/limY)² + (sz/limZ)²) …PhysXの楕円スイング円錐に対応。
    /// 1.0(100%)に張り付けば「リミットが壁」、手前で頭打ちなら「ドライブが強すぎ」。
    /// </summary>
    private void TrackWaistUtilization()
    {
        foreach (var p in _probes)
        {
            if (p.kind != Kind.Waist || p.joint == null || p.own == null || p.connected == null) continue;
            var j = p.joint;

            Quaternion rel = Quaternion.Inverse(p.connected.rotation) * p.own.rotation;
            Quaternion delta = Quaternion.Inverse(p.initialRelRot) * rel;

            // 総曲げ角（既存の取付曲げと同じ定義）
            float totalBend = Quaternion.Angle(Quaternion.identity, delta);
            if (totalBend > p.gMaxTotalBend) p.gMaxTotalBend = totalBend;

            // ジョイント座標系へ変換してから X軸まわりのTwistとそれ以外(Swing)に分解
            Quaternion dJ = Quaternion.Inverse(p.jointFrame) * delta * p.jointFrame;
            Vector3 qv = new Vector3(dJ.x, dJ.y, dJ.z);
            Vector3 proj = Vector3.Project(qv, Vector3.right);
            Quaternion twist = new Quaternion(proj.x, proj.y, proj.z, dJ.w);
            float tn = Mathf.Sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z + twist.w * twist.w);
            if (tn < 1e-6f) twist = Quaternion.identity;
            else { twist.x /= tn; twist.y /= tn; twist.z /= tn; twist.w /= tn; }
            Quaternion swing = dJ * Quaternion.Inverse(twist);

            // ツイスト角[deg]（符号付き、-180..180）
            float twistDeg = 2f * Mathf.Atan2(twist.x, twist.w) * Mathf.Rad2Deg;
            if (twistDeg > 180f) twistDeg -= 360f;
            if (twistDeg < -180f) twistDeg += 360f;

            // スイングをY/Z成分[deg]へ（回転ベクトル近似）
            swing.ToAngleAxis(out float sa, out Vector3 saxis);
            if (sa > 180f) { sa = 360f - sa; saxis = -saxis; }
            if (float.IsNaN(saxis.x)) { sa = 0f; saxis = Vector3.up; }
            float sy = sa * saxis.y;
            float sz = sa * saxis.z;

            // 円錐到達率：Limitedな軸だけで評価。Free軸は制限なし＝分母から除外。
            //   Locked軸は本来0°のはずなので、はみ出しがあればそれ自体をレポートで出す。
            bool yLim = j.angularYMotion == ConfigurableJointMotion.Limited;
            bool zLim = j.angularZMotion == ConfigurableJointMotion.Limited;
            float coneUtil = -1f;
            float limY = Mathf.Max(j.angularYLimit.limit, 0.01f);
            float limZ = Mathf.Max(j.angularZLimit.limit, 0.01f);
            if (yLim && zLim)
                coneUtil = Mathf.Sqrt((sy / limY) * (sy / limY) + (sz / limZ) * (sz / limZ));
            else if (yLim) coneUtil = Mathf.Abs(sy) / limY;
            else if (zLim) coneUtil = Mathf.Abs(sz) / limZ;

            if (coneUtil > p.gMaxConeUtil)
            {
                p.gMaxConeUtil = coneUtil;
                p.gConeSyAtMax = sy;
                p.gConeSzAtMax = sz;
            }
            if (coneUtil > p.intervalMaxConeUtil) p.intervalMaxConeUtil = coneUtil;

            // ツイスト到達率（X=Limitedのときのみ。lowは負値なので符号で分母を選ぶ）
            if (j.angularXMotion == ConfigurableJointMotion.Limited)
            {
                float tu;
                if (twistDeg >= 0f) tu = twistDeg / Mathf.Max(j.highAngularXLimit.limit, 0.01f);
                else tu = twistDeg / Mathf.Min(j.lowAngularXLimit.limit, -0.01f);
                if (tu > p.gMaxTwistUtil)
                {
                    p.gMaxTwistUtil = tu;
                    p.gTwistAtMax = twistDeg;
                }
            }
        }
    }

    /// <summary>
    /// ★ターンイベント追跡：瞬間回転速度がしきい値を超えている間＋余韻(tail)を
    /// 1つの「ターン」として扱い、その窓内でのスカート応答の最大値を記録する。
    /// 終了時にイベント単位のログを出す。「速いターンの瞬間、スカートは実際
    /// どこまで応答しているか」を直接答えるための計測。
    /// </summary>
    private void TrackTurnEvent(float instRate)
    {
        if (_elapsed < settleSeconds || !_radiusBaselineCaptured) return;

        if (instRate >= turnEventThreshold)
        {
            if (!_inTurnEvent)
            {
                _inTurnEvent = true;
                _eventStartTime = _elapsed;
                _eventPeakRate = 0f;
                _eventMaxLiftMm = 0f;
                _eventMaxRadiusMm = 0f;
                _eventMaxTotalBend = 0f;
                _eventMaxTiltDeg = 0f;
            }
            _eventLastAboveTime = _elapsed;
            if (instRate > _eventPeakRate) _eventPeakRate = instRate;
        }

        if (!_inTurnEvent) return;

        // 窓内のスカート応答を毎ステップ更新
        float waistY = _waistRef.connected.position.y;
        Vector3 center = Vector3.zero;
        int cnt = 0;
        foreach (var rp in _radiusProbes)
        {
            if (rp.rb == null) continue;
            center += rp.rb.position; cnt++;
        }
        if (cnt > 0)
        {
            center /= cnt;
            foreach (var rp in _radiusProbes)
            {
                if (rp.rb == null) continue;
                Vector3 rel = rp.rb.position - center; rel.y = 0f;
                float radMm = (rel.magnitude - rp.baselineRadius) * 1000f;
                if (radMm > _eventMaxRadiusMm) _eventMaxRadiusMm = radMm;
                float liftMm = ((rp.rb.position.y - waistY) - rp.baselineRelY) * 1000f;
                if (liftMm > _eventMaxLiftMm) _eventMaxLiftMm = liftMm;
            }
        }
        if (_totalBendProbes.Count > 0)
        {
            Quaternion waistRot = _waistRef.connected.rotation;
            foreach (var tb in _totalBendProbes)
            {
                if (tb.rb == null) continue;
                Quaternion rel = Quaternion.Inverse(waistRot) * tb.rb.rotation;
                float bend = Quaternion.Angle(tb.initialRelRot, rel);
                if (bend > _eventMaxTotalBend) _eventMaxTotalBend = bend;

                // ★傾き(スイングのみ)：相対回転で「上方向」がどれだけ倒れたか。
                //   ヨー(鉛直軸まわりの回転)は上方向の写り先を変えないため、
                //   ターン中のヨー遅れが混入しない「見た目の翻り」の実測になる。
                float tilt = Vector3.Angle(rel * Vector3.up, tb.initialRelRot * Vector3.up);
                if (tilt > _eventMaxTiltDeg) _eventMaxTiltDeg = tilt;
            }
        }

        // 余韻が切れたらイベント終了→ログ
        if (_elapsed - _eventLastAboveTime > turnEventTailSeconds)
        {
            _inTurnEvent = false;
            _turnEventCount++;
            string line = $"ターン#{_turnEventCount} t={_eventStartTime:F1}〜{_eventLastAboveTime:F1}s " +
                          $"ピーク{_eventPeakRate:F0}°/s → 窓内max: ★傾き(スイングのみ){_eventMaxTiltDeg:F1}° " +
                          $"持ち上がり{_eventMaxLiftMm:F0}mm 半径Δ{_eventMaxRadiusMm:F0}mm " +
                          $"総曲げ{_eventMaxTotalBend:F1}°(ヨー遅れ込み・参考値)";
            Debug.Log($"[JointProbe2][ターンイベント] {line}");
            _turnEventReport.AppendLine("  " + line);
            if (_eventPeakRate > _bestEventPeakRate)
            {
                _bestEventPeakRate = _eventPeakRate;
                _bestEventLiftMm = _eventMaxLiftMm;
                _bestEventBend = _eventMaxTotalBend;
                _bestEventTiltDeg = _eventMaxTiltDeg;
            }
        }
    }

    private void LateUpdate()
    {
        // ★FixedUpdate末尾で捕まえた「物理直後の値」と、ここ(描画直前)の値を比較。
        //   ズレていれば、Update/LateUpdateの間で何か(Animation・IK等)が
        //   Transformを書き換えている証拠になる。
        if (_overwriteWatchBone == null || !_watchBoneCaptured) return;

        float rotDiff = Quaternion.Angle(_afterFixedUpdateRot, _overwriteWatchBone.localRotation);
        float posDiffMm = (_overwriteWatchBone.localPosition - _afterFixedUpdatePos).magnitude * 1000f;

        if (rotDiff > _maxOverwriteDeg) _maxOverwriteDeg = rotDiff;
        if (posDiffMm > _maxOverwritePosMm) _maxOverwritePosMm = posDiffMm;
        if (rotDiff > 1f || posDiffMm > 1f) _overwriteEventCount++;

        if (_elapsed >= _nextOverwriteLog)
        {
            _nextOverwriteLog = _elapsed + logInterval;
            if (_maxOverwriteDeg > 1f || _maxOverwritePosMm > 1f)
                Debug.LogWarning($"[JointProbe2][上書き検出] t={_elapsed:F1}s 直近{logInterval:F0}秒での最大ズレ: " +
                                  $"回転{_maxOverwriteDeg:F1}° / 位置{_maxOverwritePosMm:F1}mm （{_overwriteEventCount}回検出）" +
                                  $" ← FixedUpdate後に何かがTransformを書き換えている可能性");
            else
                Debug.Log($"[JointProbe2][上書き検出] t={_elapsed:F1}s 直近{logInterval:F0}秒でズレなし（物理の結果がそのまま描画されている）");
            _maxOverwriteDeg = 0f;
            _maxOverwritePosMm = 0f;
            _overwriteEventCount = 0;
        }
    }

    /// <summary>
    /// ★Play終了時の最終レポート：取付ジョイントごとに
    /// 「設定リミット vs 実測最大」を表形式で出し、犯人①/②の判定材料をまとめる。
    /// </summary>
    private void OnDestroy()
    {
        var waist = _probes.Where(p => p.kind == Kind.Waist && p.gMaxTotalBend > 0f).ToList();
        if (waist.Count == 0) return;

        var sb = new StringBuilder();
        sb.AppendLine($"[JointProbe2][到達率 最終レポート] settle後({settleSeconds:F0}s〜{_elapsed:F1}s)の毎ステップ最大値");
        foreach (var p in waist.OrderBy(x => x.own.name))
        {
            var j = p.joint;
            string cone;
            if (p.gMaxConeUtil >= 0f)
                cone = $"円錐到達率max={p.gMaxConeUtil * 100f:F0}% " +
                       $"(その時 sy={p.gConeSyAtMax:F1}°/limY={j.angularYLimit.limit:F1}° " +
                       $"sz={p.gConeSzAtMax:F1}°/limZ={j.angularZLimit.limit:F1}°)";
            else
                cone = "円錐到達率=対象外(スイング軸に制限なし)";
            string twist;
            if (p.gMaxTwistUtil >= 0f)
                twist = $"ツイスト到達率max={p.gMaxTwistUtil * 100f:F0}% (その時 {p.gTwistAtMax:F1}°, 範囲[{j.lowAngularXLimit.limit:F1}..{j.highAngularXLimit.limit:F1}]°)";
            else
                twist = "ツイスト=対象外(X軸Limited以外)";
            sb.AppendLine($"  {p.own.name} {p.limitNote}: {cone} | {twist} | 総曲げmax={p.gMaxTotalBend:F1}°");
        }

        var utils = waist.Where(p => p.gMaxConeUtil >= 0f).Select(p => p.gMaxConeUtil * 100f).ToList();
        if (utils.Count > 0)
        {
            sb.AppendLine($"  ── 円錐到達率maxの集計: 中央値{Median(utils):F0}% / 最小{utils.Min():F0}% / 最大{utils.Max():F0}%");
            sb.AppendLine("  ★判定目安: おおむね90%以上に張り付き→犯人①リミットが壁(→ソフトリミット化へ)");
            sb.AppendLine("  ★判定目安: おおむね60%未満で頭打ち→犯人②ドライブ過強(→取付用ダイヤル拡張へ)");
            sb.AppendLine("  ★中間の場合は両方が効いている可能性あり。数値を持ち寄って相談を。");
        }

        // ★ターンイベント一覧：速い回転の瞬間にスカートがどこまで応答したかの記録
        if (_turnEventCount > 0)
        {
            sb.AppendLine($"  ── ターンイベント（瞬間{turnEventThreshold:F0}°/s超）: {_turnEventCount} 回");
            sb.Append(_turnEventReport);
            sb.AppendLine($"  ★最速ターン: {_bestEventPeakRate:F0}°/s → 傾き(スイングのみ){_bestEventTiltDeg:F1}° / 持ち上がり{_bestEventLiftMm:F0}mm / 総曲げ{_bestEventBend:F1}°(ヨー遅れ込み)");
        }
        else
        {
            sb.AppendLine($"  ── ターンイベント: 検出なし（瞬間回転が一度も{turnEventThreshold:F0}°/sを超えていない）");
        }
        Debug.Log(sb.ToString());
    }

    private void LogSummary()
    {
        // スカートの中心軸（鉛直）＝スカート剛体の重心を通る
        var skirtBodies = _probes.Where(p => p.kind != Kind.Waist).Select(p => p.own).Distinct().ToList();
        if (skirtBodies.Count == 0) return;
        Vector3 center = Vector3.zero;
        foreach (var rb in skirtBodies) center += rb.position;
        center /= skirtBodies.Count;

        var radialByRing = new Dictionary<int, List<float>>();
        var tangByRing = new Dictionary<int, List<float>>();
        var bendVertical = new List<float>();
        var bendWaist = new List<float>();

        // ★半径ベースの計測（剛体位置そのものから）。
        //   初回のLogSummary呼び出し（=t≈settleSeconds、スピン開始前）を基準半径として
        //   記録し、以降はそこからの増減[mm]を段ごとに集計する。
        float waistY = (_waistRef != null && _waistRef.connected != null) ? _waistRef.connected.position.y : 0f;
        if (!_radiusBaselineCaptured)
        {
            foreach (var rp in _radiusProbes)
            {
                if (rp.rb == null) continue;
                Vector3 rel = rp.rb.position - center; rel.y = 0f;
                rp.baselineRadius = rel.magnitude;
                // ★「翻り」＝裾の持ち上がり計測用の基準値。下半身のY位置を基準に取ることで、
                //   キャラ自体の上下動(ジャンプ等)をキャンセルし、スカート裾が体に対して
                //   どれだけ持ち上がったかだけを見る。
                rp.baselineRelY = rp.rb.position.y - waistY;
            }
            _radiusBaselineCaptured = true;
        }
        var radiusDeltaByRing = new Dictionary<int, List<float>>();
        var liftByRing = new Dictionary<int, List<float>>();
        foreach (var rp in _radiusProbes)
        {
            if (rp.rb == null) continue;
            Vector3 rel = rp.rb.position - center; rel.y = 0f;
            float delta = (rel.magnitude - rp.baselineRadius) * 1000f; // [mm]、+が外側へ広がった分
            if (!radiusDeltaByRing.ContainsKey(rp.ring)) radiusDeltaByRing[rp.ring] = new List<float>();
            radiusDeltaByRing[rp.ring].Add(delta);

            float relY = rp.rb.position.y - waistY;
            float lift = (relY - rp.baselineRelY) * 1000f; // [mm]、+が持ち上がった分
            if (!liftByRing.ContainsKey(rp.ring)) liftByRing[rp.ring] = new List<float>();
            liftByRing[rp.ring].Add(lift);
        }

        foreach (var p in _probes)
        {
            if (p.joint == null || p.own == null || p.connected == null) continue;

            if (p.kind == Kind.Lateral)
            {
                Vector3 a = p.own.transform.TransformPoint(p.joint.anchor);
                Vector3 b = p.connected.transform.TransformPoint(p.joint.connectedAnchor);
                Vector3 d = a - b; // アンカー間の変位（伸びベクトル）

                Vector3 mid = (a + b) * 0.5f;
                Vector3 radialDir = mid - center; radialDir.y = 0f;
                if (radialDir.sqrMagnitude < 1e-8f) continue;
                radialDir.Normalize();
                Vector3 tangDir = Vector3.Cross(Vector3.up, radialDir);

                int ring = p.ring;
                if (!radialByRing.ContainsKey(ring)) { radialByRing[ring] = new List<float>(); tangByRing[ring] = new List<float>(); }
                radialByRing[ring].Add(Vector3.Dot(d, radialDir) * 1000f);      // 径方向 [mm]（+が外向き）
                tangByRing[ring].Add(Mathf.Abs(Vector3.Dot(d, tangDir)) * 1000f); // 接線方向 [mm]
            }
            else
            {
                var rel = Quaternion.Inverse(p.connected.rotation) * p.own.rotation;
                float bend = Quaternion.Angle(p.initialRelRot, rel);
                if (p.kind == Kind.Vertical) bendVertical.Add(bend);
                else bendWaist.Add(bend);
            }
        }

        var sb = new StringBuilder();
        sb.Append($"[JointProbe2] t={_elapsed:F1}s ");
        foreach (var ring in radialByRing.Keys.OrderBy(r => r))
        {
            var rad = radialByRing[ring];
            var tan = tangByRing[ring];
            sb.Append($"| 段{ring}(n={rad.Count}): 径 中央値{Median(rad):+0.0;-0.0}mm/max{rad.Max():F1}mm " +
                      $"接線 中央値{Median(tan):F1}mm/max{tan.Max():F1}mm ");
        }
        foreach (var ring in radiusDeltaByRing.Keys.OrderBy(r => r))
        {
            var rd = radiusDeltaByRing[ring];
            sb.Append($"| ★半径Δ段{ring}(n={rd.Count}): 中央値{Median(rd):+0.0;-0.0}mm/max{rd.Max(v => Mathf.Abs(v)):F1}mm ");
        }
        foreach (var ring in liftByRing.Keys.OrderBy(r => r))
        {
            var lf = liftByRing[ring];
            sb.Append($"| ★持ち上がりΔ段{ring}(n={lf.Count}): 中央値{Median(lf):+0.0;-0.0}mm/max{lf.Max(v => Mathf.Abs(v)):F1}mm ");
        }
        if (bendWaist.Count > 0)
            sb.Append($"| 取付曲げ(n={bendWaist.Count}): 中央値{Median(bendWaist):F1}°/max{bendWaist.Max():F1}° ");

        // ★区間ごとの円錐到達率（毎ステップ追跡した区間max。100%=リミット張り付き）
        {
            var utils = _probes.Where(p => p.kind == Kind.Waist && p.intervalMaxConeUtil >= 0f)
                               .Select(p => p.intervalMaxConeUtil * 100f).ToList();
            if (utils.Count > 0)
                sb.Append($"| ★取付到達率(区間, n={utils.Count}): 中央値{Median(utils):F0}%/max{utils.Max():F0}% ");
            foreach (var p in _probes)
                if (p.kind == Kind.Waist) p.intervalMaxConeUtil = -1f;
        }
        if (_totalBendProbes.Count > 0 && _waistRef != null && _waistRef.connected != null)
        {
            Quaternion waistRot = _waistRef.connected.rotation;
            var totalBend = new List<float>();
            foreach (var tb in _totalBendProbes)
            {
                if (tb.rb == null) continue;
                Quaternion rel = Quaternion.Inverse(waistRot) * tb.rb.rotation;
                totalBend.Add(Quaternion.Angle(tb.initialRelRot, rel));
            }
            if (totalBend.Count > 0)
                sb.Append($"| ★総合取付曲げ(n={totalBend.Count}): 中央値{Median(totalBend):F1}°/max{totalBend.Max():F1}° ");
        }
        if (bendVertical.Count > 0)
            sb.Append($"| 縦曲げ(n={bendVertical.Count}): 中央値{Median(bendVertical):F1}°/max{bendVertical.Max():F1}° ");

        // 取付アンカーの公転実測（毎ステップ積算値の平均：エイリアスしない）
        if (_waistRef != null && _waistRef.connected != null)
        {
            float dt = _elapsed - _prevTime;
            if (dt > 1e-4f)
            {
                sb.Append($"| ★相手剛体Y回転={_yawAccum / dt:F0}°/s(平均) 瞬間ピーク={_intervalPeakYawRate:F0}°/s アンカー移動={_anchorPathAccum / dt * 1000f:F0}mm/s ");
                _yawAccum = 0f;
                _anchorPathAccum = 0f;
                _intervalPeakYawRate = 0f;
                _prevTime = _elapsed;
            }
        }
        Debug.Log(sb.ToString());
    }

    private static float Median(List<float> values)
    {
        if (values.Count == 0) return 0f;
        var sorted = values.OrderBy(v => v).ToList();
        int mid = sorted.Count / 2;
        return sorted.Count % 2 == 1 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) * 0.5f;
    }
}