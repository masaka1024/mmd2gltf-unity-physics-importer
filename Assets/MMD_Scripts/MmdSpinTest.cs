using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

/// <summary>
/// スピンテスト：キャラを一定角速度で回し続け、髪・スカートの「外向き傾き角」を実測して
/// 遠心力の再現度を診断する。
/// 物理的な期待値 tanθ = ω²r / g と比較し、実測/期待の比率で容疑者を絞る。
///
/// 使い方：
///  1. モデルのルート（Animatorが付いているオブジェクト）にこのコンポーネントを追加
///  2. Mode を選んで Play（まずは RootTransform から）
///  3. Console に1秒ごとの計測サマリーが出る（settleSeconds 経過後）
/// </summary>
public class MmdSpinTest : MonoBehaviour
{
    public enum SpinMode
    {
        RootTransform,    // (a) ルートTransformを直接回す（テレポート疑惑の検証）
        BoneTransform,    // (b) 指定ボーンのTransformを回す（Animator相当の動かし方）
        BoneMoveRotation, // (c) 指定ボーンをRigidbody.MoveRotationで回す（速度が乗る動かし方）
    }

    [Header("回し方")]
    public SpinMode mode = SpinMode.RootTransform;

    public enum SpinMotion
    {
        Constant, // 従来：一定角速度で回し続ける
        Turn,     // ★180°ターン再現：短時間の急旋回を往復で繰り返す
    }

    [Header("★ターン再現モード")]
    [Tooltip("Constant=従来の定角速度。Turn=急旋回の再現（往復）")]
    public SpinMotion motion = SpinMotion.Constant;

    [Tooltip("1回のターンで回る角度 [度]")]
    public float turnAngleDeg = 180f;

    [Tooltip("ターンの所要時間 [秒]。0.25〜0.5が本家ダンス相当")]
    public float turnDurationSeconds = 0.3f;

    [Tooltip("ターン間の休止 [秒]（スカートが静止に戻るのを待つ）")]
    public float turnPauseSeconds = 1.5f;

    [Tooltip("1条件あたりのターン回数（往復で交互方向）")]
    public int turnRepeats = 4;

    [Tooltip("ONでターンごとに回転方向を反転（往復）。OFFで毎回同方向")]
    public bool alternateDirection = true;

    [Tooltip("ターン終了後も計測を続ける余韻 [秒]（ホイップの山を含めるため）")]
    public float turnTailSeconds = 0.6f;

    [System.Serializable]
    public class SweepCondition
    {
        public string label = "A:基準";
        [Tooltip("ソフトリミットばねの倍率（1=現行ビルド値）")]
        public float softSpringMul = 1f;
        [Tooltip("角度リミットの倍率（ヨー軸は自動除外。1=現行）")]
        public float limitScaleMul = 1f;
        [Tooltip("横リング直線リミットの倍率（1=現行）")]
        public float linearMul = 1f;
    }

    [Header("★パラメータ自動スイープ")]
    [Tooltip("ONでsweepConditionsを順に適用し、各条件でターンを実行して結果をランキング表示。OFFなら現行設定のままターンのみ")]
    public bool enableSweep = false;

    [Tooltip("条件切替後、計測を始める前の整定時間 [秒]")]
    public float resettleSeconds = 2f;

    [Tooltip("スイープ条件のリスト（空ならStart時に既定8条件を自動生成）")]
    public List<SweepCondition> sweepConditions = new List<SweepCondition>();

    [Tooltip("BoneTransform / BoneMoveRotation で回すボーン（上半身や頭など）。未指定ならこのオブジェクト自身")]
    public Transform spinBone;

    [Tooltip("回転速度 [度/秒]。360 = 毎秒1回転")]
    public float degreesPerSecond = 360f;

    [Tooltip("テスト中はAnimatorを無効化する（回転と姿勢が喧嘩しないように）")]
    public bool disableAnimator = true;

    [Tooltip("ONにすると、スカートのジョイントが実際に繋がっている相手剛体を自動で探してスピン対象にする（同名オブジェクトの選び間違いを排除）。spinBoneの手動指定より優先")]
    public bool autoFindSkirtAnchor = false;

    [Header("計測")]
    [Tooltip("回し始めてから計測を開始するまでの秒数（定常状態待ち。Warmupの0.2秒も含めて余裕を持たせる）")]
    public float settleSeconds = 3f;

    [Tooltip("ログ出力の間隔 [秒]")]
    public float logInterval = 1f;

    [Tooltip("期待値計算に使う重力の大きさ [m/s²]。0 なら Physics.gravity をそのまま使う。MmdGravity等で独自重力にしている場合はその値(例: 7.85)を入れる")]
    public float gravityOverride = 0f;

    private class Segment
    {
        public string group;
        public Rigidbody rb;
        public Transform tip; // 傾きの向きを測る先端（最初の子ボーン。無ければ null）
    }

    private readonly List<Segment> _segments = new List<Segment>();
    private float _elapsed;
    private float _nextLog;
    private Transform _spinTarget;

    // ── ★ターン/スイープの内部状態 ──
    private enum TPhase { Resettle, Turning, Tail, Pause, Finished }
    private TPhase _tphase = TPhase.Resettle;
    private float _phaseT;
    private int _condIdx;
    private int _turnIdx;
    private float _turnDir = 1f;
    private float _turnMaxTilt, _turnMaxHemMm;
    private float _condBestTilt, _condBestHemMm;
    private readonly List<(string label, float tilt, float hem)> _results = new List<(string, float, float)>();

    // 実行中に書き換えるジョイントの初期値（各条件はこのベースライン×倍率で適用）
    private class JointBaseline
    {
        public ConfigurableJoint j;
        public bool soft;      // 取付＋縦（ソフトリミット対象）
        public bool lateral;   // 横リング（直線リミット対象）
        public float limY, limZ, lin;
        public SoftJointLimitSpring spring;
        public int yawAxis;    // 0=X,1=Y,2=Z（角度倍率から除外する軸）
    }
    private readonly List<JointBaseline> _jointBase = new List<JointBaseline>();
    private readonly List<(Rigidbody rb, float baseR)> _hem = new List<(Rigidbody, float)>();
    private int _hemRing = -1;

    private void Start()
    {
        _spinTarget = (mode == SpinMode.RootTransform || spinBone == null) ? transform : spinBone;

        // 自動特定：スカートのジョイントが繋がっている相手剛体そのものをターゲットにする
        if (autoFindSkirtAnchor && mode != SpinMode.RootTransform)
        {
            foreach (var j in GetComponentsInChildren<ConfigurableJoint>())
            {
                var rb = j.GetComponent<Rigidbody>();
                if (rb != null && rb.name.Contains("スカート") && j.connectedBody != null)
                {
                    _spinTarget = j.connectedBody.transform;
                    break;
                }
            }
        }

        if (disableAnimator)
        {
            var animators = GetComponentsInChildren<Animator>(true);
            foreach (var animator in animators)
                animator.enabled = false;
            var legacyAnims = GetComponentsInChildren<Animation>(true);
            foreach (var anim in legacyAnims)
            {
                anim.Stop();
                anim.enabled = false;
            }
            Debug.Log($"[SpinTest] Animator {animators.Length}個 / Animation(レガシー) {legacyAnims.Length}個 を無効化しました");
        }

        // 注意：MmdPhysicsWarmupが開始直後は揺れ物を一時Kinematicにするため、
        // ここではKinematicかどうかを見ずに全剛体を収集し、計測時に除外する
        foreach (var rb in GetComponentsInChildren<Rigidbody>())
        {
            var t = rb.transform;
            _segments.Add(new Segment
            {
                group = GroupOf(t.name),
                rb = rb,
                tip = t.childCount > 0 ? t.GetChild(0) : null,
            });
        }

        Debug.Log($"[SpinTest] mode={mode} target={GetPath(_spinTarget)} " +
                  $"omega={degreesPerSecond}deg/s 収集剛体={_segments.Count}件(Kinematic含む) " +
                  $"g={GravityMagnitude():F2}m/s2" +
                  (autoFindSkirtAnchor && mode != SpinMode.RootTransform ? " ／スカートのアンカー剛体を自動特定しました" : ""));

        if (motion == SpinMotion.Turn) SetupTurnAndSweep();
    }

    // ── ★ターン/スイープの準備 ──
    private void SetupTurnAndSweep()
    {
        // 既定8条件（リスト未設定時）。倍率はすべて「現行ビルド値に対して」
        if (enableSweep && sweepConditions.Count == 0)
        {
            sweepConditions = new List<SweepCondition>
            {
                new SweepCondition { label = "A:基準",              softSpringMul = 1f,   limitScaleMul = 1f,   linearMul = 1f },
                new SweepCondition { label = "B:ばね×0.4",          softSpringMul = 0.4f, limitScaleMul = 1f,   linearMul = 1f },
                new SweepCondition { label = "C:ばね×2",            softSpringMul = 2f,   limitScaleMul = 1f,   linearMul = 1f },
                new SweepCondition { label = "D:壁×1.5",            softSpringMul = 1f,   limitScaleMul = 1.5f, linearMul = 1f },
                new SweepCondition { label = "E:壁×1.5+ばね×0.4",   softSpringMul = 0.4f, limitScaleMul = 1.5f, linearMul = 1f },
                new SweepCondition { label = "F:直線×1.33",         softSpringMul = 1f,   limitScaleMul = 1f,   linearMul = 1.33f },
                new SweepCondition { label = "G:直線×0.67",         softSpringMul = 1f,   limitScaleMul = 1f,   linearMul = 0.67f },
                new SweepCondition { label = "H:全部盛り",          softSpringMul = 0.4f, limitScaleMul = 1.5f, linearMul = 1.33f },
            };
        }
        if (!enableSweep)
            sweepConditions = new List<SweepCondition> { new SweepCondition { label = "現行設定" } };

        // ジョイントのベースライン収集（分類はインポーター/Probeと同じルール）
        var ringRe = new System.Text.RegularExpressions.Regex(@"スカート_(\d+)_");
        foreach (var j in GetComponentsInChildren<ConfigurableJoint>())
        {
            var own = j.GetComponent<Rigidbody>();
            var conn = j.connectedBody;
            if (own == null || conn == null) continue;
            bool a = own.name.Contains("スカート");
            bool b = conn.name.Contains("スカート");
            if (!a && !b) continue;

            bool linLocked = j.xMotion == ConfigurableJointMotion.Locked
                          && j.yMotion == ConfigurableJointMotion.Locked
                          && j.zMotion == ConfigurableJointMotion.Locked;
            var jb = new JointBaseline
            {
                j = j,
                soft = (a != b) || (a && b && linLocked),
                lateral = a && b && !linLocked,
                limY = j.angularYLimit.limit,
                limZ = j.angularZLimit.limit,
                lin = j.linearLimit.limit,
                spring = j.angularYZLimitSpring,
            };
            // ヨー軸検出（インポーターと同じ：鉛直に最も近い角度軸）
            Quaternion wr = own.transform.rotation;
            float dX = Mathf.Abs(Vector3.Dot((wr * j.axis).normalized, Vector3.up));
            float dY = Mathf.Abs(Vector3.Dot((wr * j.secondaryAxis).normalized, Vector3.up));
            float dZ = Mathf.Abs(Vector3.Dot((wr * Vector3.Cross(j.axis, j.secondaryAxis)).normalized, Vector3.up));
            jb.yawAxis = (dX >= dY && dX >= dZ) ? 0 : (dY >= dZ ? 1 : 2);
            _jointBase.Add(jb);
        }

        // 裾リング（最大段番号）の剛体を特定
        foreach (var seg in _segments)
        {
            if (seg.rb == null) continue;
            var m = ringRe.Match(seg.rb.name);
            if (!m.Success) continue;
            int ring = int.Parse(m.Groups[1].Value);
            if (ring > _hemRing) _hemRing = ring;
        }

        ApplyCondition(sweepConditions[0]);
        _tphase = TPhase.Resettle; _phaseT = 0f; _condIdx = 0; _turnIdx = 0; _turnDir = 1f;
        _condBestTilt = 0f; _condBestHemMm = 0f;
        Debug.Log($"[SpinTest][ターン] 開始: {sweepConditions.Count}条件 × {turnRepeats}ターン " +
                  $"({turnAngleDeg:F0}°/{turnDurationSeconds:F2}s, 休止{turnPauseSeconds:F1}s) " +
                  $"対象ジョイント: ソフト{_jointBase.Count(x => x.soft)}本/横{_jointBase.Count(x => x.lateral)}本, 裾=段{_hemRing}");
    }

    private void ApplyCondition(SweepCondition c)
    {
        foreach (var jb in _jointBase)
        {
            if (jb.j == null) continue;
            if (jb.soft)
            {
                var sp = jb.spring;
                sp.spring *= c.softSpringMul;
                jb.j.angularYZLimitSpring = sp;
                var y = jb.j.angularYLimit;
                y.limit = jb.limY * (jb.yawAxis == 1 ? 1f : c.limitScaleMul);
                jb.j.angularYLimit = y;
                var z = jb.j.angularZLimit;
                z.limit = jb.limZ * (jb.yawAxis == 2 ? 1f : c.limitScaleMul);
                jb.j.angularZLimit = z;
            }
            if (jb.lateral)
            {
                var l = jb.j.linearLimit;
                l.limit = jb.lin * c.linearMul;
                jb.j.linearLimit = l;
            }
        }
        Debug.Log($"[SpinTest][スイープ] 条件 {c.label} を適用（ばね×{c.softSpringMul:F2} 壁×{c.limitScaleMul:F2} 直線×{c.linearMul:F2}）");
    }

    private static string GetPath(Transform t)
    {
        var s = t.name;
        while (t.parent != null) { t = t.parent; s = t.name + "/" + s; }
        return s;
    }

    private Quaternion _lastWrittenRot;
    private bool _hasWritten;
    private float _revertAccum;
    private float _revertPrevTime;

    private void FixedUpdate()
    {
        if (motion == SpinMotion.Turn)
        {
            TurnFixedUpdate();
            return;
        }

        float delta = degreesPerSecond * Time.fixedDeltaTime;
        var spin = Quaternion.AngleAxis(delta, Vector3.up);

        // 巻き戻し検出：前ステップの書き込み後から今までの間に、誰かが回転を書き換えたか
        if (_hasWritten)
            _revertAccum += Quaternion.Angle(_lastWrittenRot, _spinTarget.rotation);

        switch (mode)
        {
            case SpinMode.RootTransform:
                transform.rotation = spin * transform.rotation;
                break;
            case SpinMode.BoneTransform:
                _spinTarget.rotation = spin * _spinTarget.rotation;
                break;
            case SpinMode.BoneMoveRotation:
                var krb = _spinTarget.GetComponent<Rigidbody>();
                if (krb != null) krb.MoveRotation(spin * krb.rotation);
                else _spinTarget.rotation = spin * _spinTarget.rotation;
                break;
        }

        _lastWrittenRot = _spinTarget.rotation;
        _hasWritten = true;

        _elapsed += Time.fixedDeltaTime;
        if (_elapsed >= settleSeconds && _elapsed >= _nextLog)
        {
            _nextLog = _elapsed + logInterval;
            LogSummary();
        }
    }

    // ── ★ターン再現の状態機械 ──
    private void TurnFixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        _elapsed += dt;
        _phaseT += dt;

        switch (_tphase)
        {
            case TPhase.Resettle:
                if (_phaseT >= Mathf.Max(resettleSeconds, settleSeconds - _elapsed + _phaseT))
                {
                    CaptureHemBaseline();
                    BeginTurn();
                }
                break;

            case TPhase.Turning:
                WriteSpin(_turnDir * (turnAngleDeg / Mathf.Max(turnDurationSeconds, 0.01f)) * dt);
                MeasureTurnWindow();
                if (_phaseT >= turnDurationSeconds)
                {
                    _tphase = TPhase.Tail; _phaseT = 0f;
                }
                break;

            case TPhase.Tail:
                MeasureTurnWindow();
                if (_phaseT >= turnTailSeconds) EndTurn();
                break;

            case TPhase.Pause:
                if (_phaseT >= turnPauseSeconds)
                {
                    if (_turnIdx < turnRepeats) BeginTurn();
                    else NextConditionOrFinish();
                }
                break;

            case TPhase.Finished:
                break;
        }
    }

    private void WriteSpin(float deltaDeg)
    {
        var spin = Quaternion.AngleAxis(deltaDeg, Vector3.up);
        switch (mode)
        {
            case SpinMode.RootTransform:
                transform.rotation = spin * transform.rotation;
                break;
            case SpinMode.BoneTransform:
                _spinTarget.rotation = spin * _spinTarget.rotation;
                break;
            case SpinMode.BoneMoveRotation:
                var krb = _spinTarget.GetComponent<Rigidbody>();
                if (krb != null) krb.MoveRotation(spin * krb.rotation);
                else _spinTarget.rotation = spin * _spinTarget.rotation;
                break;
        }
    }

    private void BeginTurn()
    {
        _tphase = TPhase.Turning; _phaseT = 0f;
        _turnIdx++;
        _turnMaxTilt = 0f; _turnMaxHemMm = 0f;
    }

    private void EndTurn()
    {
        var c = sweepConditions[_condIdx];
        Debug.Log($"[SpinTest][ターン] {c.label} #{_turnIdx} ({(_turnDir > 0 ? "+" : "-")}{turnAngleDeg:F0}°/{turnDurationSeconds:F2}s) " +
                  $"→ 傾きmax {_turnMaxTilt:F1}° 裾Δmax {_turnMaxHemMm:F0}mm");
        if (_turnMaxTilt > _condBestTilt) _condBestTilt = _turnMaxTilt;
        if (_turnMaxHemMm > _condBestHemMm) _condBestHemMm = _turnMaxHemMm;
        if (alternateDirection) _turnDir = -_turnDir;
        _tphase = TPhase.Pause; _phaseT = 0f;
    }

    private void NextConditionOrFinish()
    {
        var c = sweepConditions[_condIdx];
        _results.Add((c.label, _condBestTilt, _condBestHemMm));
        Debug.Log($"[SpinTest][スイープ] 条件 {c.label} 完了 → 傾きbest {_condBestTilt:F1}° / 裾Δbest {_condBestHemMm:F0}mm");

        _condIdx++;
        if (_condIdx < sweepConditions.Count)
        {
            ApplyCondition(sweepConditions[_condIdx]);
            _turnIdx = 0; _condBestTilt = 0f; _condBestHemMm = 0f;
            _tphase = TPhase.Resettle; _phaseT = 0f;
        }
        else
        {
            var sb = new StringBuilder();
            sb.AppendLine("[SpinTest][スイープ] ★最終ランキング（傾きbest順）");
            foreach (var r in _results.OrderByDescending(x => x.tilt))
                sb.AppendLine($"  {r.label}: 傾き {r.tilt:F1}° / 裾Δ {r.hem:F0}mm");
            sb.AppendLine("  参考: 本家ターン実測の傾きは40〜63°（同条件比較はダンス側の同時刻イベント対決で）");
            Debug.Log(sb.ToString());
            _tphase = TPhase.Finished;
        }
    }

    private void CaptureHemBaseline()
    {
        _hem.Clear();
        if (_hemRing < 0) return;
        string prefix = $"スカート_{_hemRing}_";
        Vector3 axis = _spinTarget.position;
        foreach (var seg in _segments)
        {
            if (seg.rb == null || !seg.rb.name.Contains(prefix)) continue;
            Vector3 h = seg.rb.position - axis; h.y = 0f;
            _hem.Add((seg.rb, h.magnitude));
        }
    }

    private void MeasureTurnWindow()
    {
        // 傾き：スカート剛体の「真下からの角度」（従来の傾き定義と同一）
        foreach (var seg in _segments)
        {
            if (seg.rb == null || seg.rb.isKinematic || seg.group != "スカート") continue;
            Vector3 v;
            if (seg.tip != null) v = seg.tip.position - seg.rb.position;
            else if (seg.rb.transform.parent != null) v = seg.rb.position - seg.rb.transform.parent.position;
            else continue;
            if (v.sqrMagnitude < 1e-10f) continue;
            float tilt = Vector3.Angle(v, Vector3.down);
            if (tilt > _turnMaxTilt) _turnMaxTilt = tilt;
        }
        // 裾Δ：裾リング剛体の回転軸からの水平距離の増分
        Vector3 axis = _spinTarget.position;
        foreach (var (rb, baseR) in _hem)
        {
            if (rb == null) continue;
            Vector3 h = rb.position - axis; h.y = 0f;
            float d = (h.magnitude - baseR) * 1000f;
            if (d > _turnMaxHemMm) _turnMaxHemMm = d;
        }
    }

    private void LogSummary()
    {
        float g = GravityMagnitude();
        float omega = degreesPerSecond * Mathf.Deg2Rad;
        Vector3 axisPoint = _spinTarget.position;

        var byGroup = new Dictionary<string, (List<float> tilt, List<float> expect)>();

        int measured = 0;
        foreach (var seg in _segments)
        {
            if (seg.rb == null || seg.rb.isKinematic) continue; // 体側(常時Kinematic)とWarmup中は除外
            // セグメントの向き：自ボーン→最初の子。子が無い末端は 親→自ボーン で代用
            Vector3 v;
            if (seg.tip != null) v = seg.tip.position - seg.rb.position;
            else if (seg.rb.transform.parent != null) v = seg.rb.position - seg.rb.transform.parent.position;
            else continue;
            if (v.sqrMagnitude < 1e-10f) continue;

            // 実測：真下からの傾き角
            float tilt = Vector3.Angle(v, Vector3.down);

            // 期待値：tanθ = ω²r / g （rは回転軸からの水平距離）
            Vector3 mid = seg.rb.position + v * 0.5f;
            Vector3 horiz = mid - axisPoint; horiz.y = 0f;
            float r = horiz.magnitude;
            float expect = Mathf.Atan2(omega * omega * r, g) * Mathf.Rad2Deg;

            if (!byGroup.TryGetValue(seg.group, out var lists))
            {
                lists = (new List<float>(), new List<float>());
                byGroup[seg.group] = lists;
            }
            lists.tilt.Add(tilt);
            lists.expect.Add(expect);
            measured++;
        }

        if (measured == 0)
        {
            Debug.LogWarning($"[SpinTest] t={_elapsed:F1}s 計測対象0件です。" +
                             $"収集済み剛体={_segments.Count}件（全てKinematicの可能性）。" +
                             $"スクリプトのアタッチ先が物理剛体の親階層か確認してください");
            return;
        }

        var sb = new StringBuilder();
        sb.Append($"[SpinTest] t={_elapsed:F1}s mode={mode} n={measured} ");
        float revertDt = _elapsed - _revertPrevTime;
        if (revertDt > 1e-4f)
        {
            sb.Append($"★巻き戻し検知={_revertAccum / revertDt:F0}°/s ");
            _revertAccum = 0f;
            _revertPrevTime = _elapsed;
        }
        foreach (var kv in byGroup.OrderBy(k => k.Key))
        {
            float tiltMed = Median(kv.Value.tilt);
            float tiltMax = kv.Value.tilt.Max();
            float expMed = Median(kv.Value.expect);
            float ratio = expMed > 0.5f ? tiltMed / expMed : -1f;
            sb.Append($"| {kv.Key}: 実測中央値{tiltMed:F1}°(max{tiltMax:F1}°) 期待{expMed:F1}° 比率{ratio:F2} ");
        }
        Debug.Log(sb.ToString());
    }

    private float GravityMagnitude()
    {
        return gravityOverride > 0f ? gravityOverride : Physics.gravity.magnitude;
    }

    private static float Median(List<float> values)
    {
        if (values.Count == 0) return 0f;
        var sorted = values.OrderBy(v => v).ToList();
        int mid = sorted.Count / 2;
        return sorted.Count % 2 == 1 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) * 0.5f;
    }

    private static string GroupOf(string name)
    {
        if (name.Contains("スカート")) return "スカート";
        if (name.Contains("モミアゲ") || name.Contains("もみあげ")) return "モミアゲ";
        if (name.Contains("前髪") || name.Contains("髪F")) return "前髪";
        if (name.Contains("髪")) return "後ろ髪";
        return "その他";
    }
}