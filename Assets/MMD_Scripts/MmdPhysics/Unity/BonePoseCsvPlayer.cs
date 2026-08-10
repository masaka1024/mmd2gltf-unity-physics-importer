// ===========================================================================
// ボーン姿勢CSV 再生コンポーネント (目視確認用)。
// MMDでベイクしたのボーン世界姿勢CSVで PMX 物理を駆動し、ヘッドレス(HeadlessDriver)と
// 同一入力・同一ロジックで同じ動きを Unity 上に再現する。さらに、CSVに含まれるMMDの
// スカートボーン姿勢から「MMDのスカート剛体」をゴースト(別色)で重ね描きし、自前物理との
// ズレを目視で比較できるようにする。
//
// 操作は Inspector の右クリック(ContextMenu)で行う (Input/GUI 非依存):
//   Play / Pause / Step Forward(+1) / Step Back(-1) / Jump to Frame / 窓先頭/末尾へ
// 特に窓6 (F2440〜F2470) をコマ送りで確認するため、WindowStart/End とジャンプを用意。
//
// 注意: 物理は逆再生できないため、後退やジャンプは「フレーム0から目標まで再シミュレーション」
//   する (7000フレームでも一瞬)。前進1フレームは差分で進むso安価。
// ===========================================================================
using System.Collections.Generic;
using UnityEngine;
using BulletPhysics.Pmx;

namespace BulletPhysics.Unity
{
    public sealed class BonePoseCsvPlayer : MonoBehaviour
    {
        [Header("入力 (各自の環境のパスを Inspector で設定)")]
        [Tooltip("読み込む .pmx ファイルのパス。空なら何もせず警告のみ (落ちない)")]
        public string PmxPath = "";
        [Tooltip("MMDでベイクしたボーン世界姿勢CSVのパス。空/未存在なら物理のみ動きゴースト無し (落ちない)")]
        public string BoneCsvPath = "";

        [Header("Solver (リファレンス=実効1/60: FTS=1/30, SubSteps=2)")]
        public float Gravity = 98f;
        public int SolverIterations = 10;
        // リファレンス刻みに合わせる (ビューアの表示が解析・新ベースラインと一致するように)。
        // FixedTimeStep は 30fps 入力に合わせ 1/30 のまま、SubSteps で刻む。詳細は DESIGN.md。
        public int SubSteps = 2;
        public float FixedTimeStep = 1f / 30f;
        [Tooltip("計測開始前にフレーム0姿勢で空回しするステップ数 (バインド姿勢からの沈み込み過渡を除く)")]
        public int WarmupSteps = 60;

        [Header("表示")]
        [Tooltip("エンジン(PMXネイティブ単位) -> Unity 配置スケール")]
        public float UnitScale = 0.08f;
        public bool DrawSelf = true;
        [Tooltip("MMDのスカート剛体をゴースト(マゼンタ)で重ね描き")]
        public bool DrawReferenceGhost = true;
        [Tooltip("ゴーストをスカート(CSVに含まれるボーン)に限定")]
        public bool SkirtOnlyGhost = true;

        [Header("再生")]
        public bool Playing = false;
        [Tooltip("再生速度 (実時間の何倍速でフレームを進めるか。30=等速)")]
        public float PlaybackFps = 30f;
        [Tooltip("現在ワールドが到達しているフレーム (表示専用。移動は下の ContextMenu で)")]
        public int Frame = 0;

        [Header("窓6 コマ送り (自前92.2°/MMD62.9°の確認)")]
        public int WindowStart = 2440;
        public int WindowEnd = 2470;

        [Header("貫入の可視化 (表示専用・物理に影響しない)")]
        [Tooltip("貫入している剛体を橙〜赤で塗る (深さで濃淡)")]
        public bool HighlightPenetration = true;
        [Tooltip("接触点(球)と法線(線分, 長さ=深さ)を描く")]
        public bool DrawContacts = true;
        [Tooltip("接触法線の線分長 = 貫入深さ × この倍率 (見た目調整用)")]
        public float ContactNormalScale = 3f;
        // 深さ閾値の根拠 (タスクA/C 実測, PMXネイティブ単位): 貫入は平均~0.03 で常在する
        // 浅い接触が多数あるため 0.05 未満は無視 (橙にすると常時真っ赤になる)。深いクラスタは
        // ~0.2-0.5、最大~2.1。0.05-0.3 を橙、0.3 以上を赤とする (スカート×太もも~0.95 は赤)。
        private const float PenIgnore = 0.05f, PenDeep = 0.3f;

        // 貫入フレーム候補 (全編再測定で再選定)。スカート×太ももの深い食い込みは F0-120 の
        // 初期整定に集中し(全編最深=F8)、以降はほぼ0。中盤最深=F2944(0.10), 最浅代表=F3500(0),
        // 全編最深(髪スパイク)=F2889(2.15)。先頭に主対象(スカート×太もも 初期整定)、後方に中盤/浅/髪。
        [Header("貫入フレーム候補 (全編再測定で再選定)")]
        public int[] CandidateFrames = { 8, 4, 9, 18, 60, 120, 2944, 3500, 2889 };
        public int CandidateIndex = 0;

        // 貫入表示のキャッシュ (物理ステップ後に更新, 表示専用)。
        private struct PenContact { public Vector3 pos; public Vector3 normal; public float depth; }
        private readonly List<PenContact> _penContacts = new();
        private readonly Dictionary<RigidBody, float> _bodyPenDepth = new();
        private readonly List<ContactPoint> _detBuf = new();
        private float _maxPenDepth; private string _maxPenA = "", _maxPenB = "";

        private PmxPhysicsBuilder _builder;
        private PmxPhysicsModel _model;
        private BonePoseCsvSource _csv;
        private List<(BoneLink link, string bone)> _driven = new();
        private int _simFrame = -1;   // ワールドが到達しているフレーム
        private float _accum;

        void Start() { Reload(); }

        [ContextMenu("Reload (PMX+CSV 再読込)")]
        public void Reload()
        {
            if (string.IsNullOrEmpty(PmxPath)) { Debug.LogWarning("[CsvPlayer] PmxPath 未設定"); return; }
            _model = PmxReader.LoadFile(PmxPath);
            _csv = BonePoseCsvSource.Load(BoneCsvPath);
            if (_csv == null) Debug.LogWarning($"[CsvPlayer] CSVが読めません: {BoneCsvPath}");
            RewindTo(Frame);
        }

        private void BuildWorld()
        {
            _builder = PmxPhysicsBuilder.Build(_model);
            var w = _builder.World;
            w.Gravity = new Vec3(0f, -Gravity, 0f);
            w.SolverIterations = SolverIterations;
            w.SubSteps = SubSteps;
            w.FixedTimeStep = FixedTimeStep;

            _driven.Clear();
            if (_csv == null) return;
            foreach (var link in _builder.BoneLinks)
            {
                if (link.Mode != PhysicsMode.BoneFollow) continue;
                if (link.BoneIndex < 0 || link.BoneIndex >= _model.BoneNames.Count) continue;
                string b = _model.BoneNames[link.BoneIndex];
                if (_csv.HasBone(b)) _driven.Add((link, b));
            }
        }

        private void ApplyPose(int f)
        {
            // 駆動式は共通ヘルパへ集約 (2026-08-09 hairfid誤配置事故の再発防止。式は同一)。
            _builder.ApplyKinematicTargets(bi =>
                (bi >= 0 && bi < _model.BoneNames.Count && _csv != null && _csv.TryGet(f, _model.BoneNames[bi], out var bw)) ? (RigidTransform?)bw : null);
        }

        // ボーンindex → フレーム0のCSV姿勢 (在れば)。FK-restリセットの駆動姿勢源。
        // 物理ボーン(スカート等)のCSV姿勢はFKヘルパ側で無視され親から前計算される。
        private RigidTransform? DrivenBoneWorldAtFrame0(int boneIndex)
        {
            if (_csv == null || boneIndex < 0 || boneIndex >= _model.BoneNames.Count) return null;
            if (_csv.TryGet(0, _model.BoneNames[boneIndex], out var bw)) return bw;
            return null;
        }

        /// <summary>ワールドを作り直し、フレーム0のウォームアップから target まで再シミュレーションする。</summary>
        public void RewindTo(int target)
        {
            if (_model == null) return;
            int last = _csv != null ? _csv.FrameCount - 1 : 0;
            target = System.Math.Clamp(target, 0, System.Math.Max(0, last));
            BuildWorld();
            ApplyPose(0);
            // 物理開始前に FK-rest で全剛体をボーン姿勢へ整合させる (MMDの物理リセット相当)。
            // CSVは入力ボーン(駆動)と物理ボーン(スカート等=物理結果)を両方含むが、FKヘルパが
            // 物理ボーンのCSV姿勢を無視して親から前計算するので過拘束発散しない。
            _builder.ResetBodiesToBonePoseFk(DrivenBoneWorldAtFrame0);
            for (int s = 0; s < WarmupSteps; s++) _builder.World.StepSimulation(FixedTimeStep);
            for (int f = 0; f <= target; f++) { ApplyPose(f); _builder.World.StepSimulation(FixedTimeStep); }
            _simFrame = target;
            Frame = target;
            ComputePenetrations();
        }

        [ContextMenu("Step Forward (+1)")]
        public void StepForward()
        {
            if (_builder == null || _csv == null) return;
            if (_simFrame + 1 >= _csv.FrameCount) { Playing = false; return; }
            int f = _simFrame + 1;
            ApplyPose(f);
            _builder.World.StepSimulation(FixedTimeStep);
            _simFrame = f;
            Frame = f;
            ComputePenetrations();
        }

        [ContextMenu("Step Back (-1)")]
        public void StepBack()
        {
            if (_simFrame > 0) RewindTo(_simFrame - 1);
        }

        [ContextMenu("Play")] public void Play() { Playing = true; }
        [ContextMenu("Pause")] public void Pause() { Playing = false; }
        [ContextMenu("Jump to Frame (Frame値へ)")] public void JumpToFrame() { RewindTo(Frame); }
        [ContextMenu("Jump to Window Start (窓先頭)")] public void JumpWindowStart() { RewindTo(WindowStart); }
        [ContextMenu("Jump to Window End (窓末尾)")] public void JumpWindowEnd() { RewindTo(WindowEnd); }

        [ContextMenu("Jump to Next Candidate (貫入候補→次)")]
        public void JumpNextCandidate()
        {
            if (CandidateFrames == null || CandidateFrames.Length == 0) return;
            CandidateIndex = (CandidateIndex + 1) % CandidateFrames.Length;
            RewindTo(CandidateFrames[CandidateIndex]);
        }

        [ContextMenu("Jump to Prev Candidate (貫入候補→前)")]
        public void JumpPrevCandidate()
        {
            if (CandidateFrames == null || CandidateFrames.Length == 0) return;
            CandidateIndex = (CandidateIndex - 1 + CandidateFrames.Length) % CandidateFrames.Length;
            RewindTo(CandidateFrames[CandidateIndex]);
        }

        // 表示専用: 現在の剛体配置から貫入接触を検出しキャッシュする (物理には一切影響しない)。
        // 剛体117程度なので AABB 枝刈り付き O(n^2) narrowphase で十分軽い。エンジンと同じ
        // 衝突フィルタ(ShouldCollide)と narrowphase(GjkEpa.Detect)を使う。
        private void ComputePenetrations()
        {
            _penContacts.Clear(); _bodyPenDepth.Clear(); _maxPenDepth = 0; _maxPenA = ""; _maxPenB = "";
            if (_builder == null) return;
            var bodies = _builder.Bodies; int n = bodies.Count;
            var aabbs = new Aabb[n];
            for (int i = 0; i < n; i++) aabbs[i] = bodies[i].ComputeAabb();
            for (int i = 0; i < n; i++)
                for (int k = i + 1; k < n; k++)
                {
                    var a = bodies[i]; var b = bodies[k];
                    if (a.IsStaticOrKinematic && b.IsStaticOrKinematic) continue;
                    if (!PhysicsWorld.ShouldCollide(a, b)) continue;
                    if (!aabbs[i].Intersects(ref aabbs[k])) continue;
                    _detBuf.Clear(); GjkEpa.Detect(a, b, _detBuf);
                    foreach (var cp in _detBuf)
                    {
                        float depth = -cp.Distance;
                        if (depth <= 0f) continue;
                        AccumBodyDepth(a, depth); AccumBodyDepth(b, depth);
                        var mid = (cp.PositionWorldA + cp.PositionWorldB) * 0.5f;
                        _penContacts.Add(new PenContact { pos = MmdToUnityPos(mid), normal = MmdToUnityDir(cp.Normal), depth = depth });
                        if (depth > _maxPenDepth) { _maxPenDepth = depth; _maxPenA = a.Name; _maxPenB = b.Name; }
                    }
                }
        }

        private void AccumBodyDepth(RigidBody b, float depth)
        {
            if (!_bodyPenDepth.TryGetValue(b, out float cur) || depth > cur) _bodyPenDepth[b] = depth;
        }

        // 深さ→色 (0.05-0.3=橙, 0.3以上=赤)。閾値の根拠はフィールド宣言部コメント参照。
        private static Color PenColor(float depth) =>
            depth >= PenDeep ? Color.red : new Color(1f, 0.55f, 0f, 1f);

        void FixedUpdate()
        {
            if (!Playing || _builder == null || _csv == null) return;
            _accum += Time.fixedDeltaTime * PlaybackFps;
            int steps = (int)_accum;
            _accum -= steps;
            for (int i = 0; i < steps; i++)
            {
                if (_simFrame + 1 >= _csv.FrameCount) { Playing = false; break; }
                StepForward();
            }
        }

        // ---- 座標変換 (MmdPhysicsBehaviour と同一。Z反転なし=単位スケールのみの等長変換) ----
        private Vector3 MmdToUnityPos(Vec3 v) => new(v.x * UnitScale, v.y * UnitScale, v.z * UnitScale);
        private static Quaternion MmdToUnityRot(Quat q) => new(q.x, q.y, q.z, q.w);
        // 方向 (スケール無し。二重ReverseZ相殺後はZ反転不要)。
        private static Vector3 MmdToUnityDir(Vec3 v) => new(v.x, v.y, v.z);

        void OnDrawGizmos()
        {
            if (_builder == null) return;

            // 自前物理の剛体。通常は BoneFollow=cyan, Dynamic=green, Bone合わせ=yellow。
            // 貫入している剛体は HighlightPenetration で橙〜赤に上書き (深さで濃淡)。
            if (DrawSelf)
                foreach (var body in _builder.Bodies)
                {
                    Color col = body.Mode == PhysicsMode.BoneFollow ? Color.cyan
                        : (body.Mode == PhysicsMode.Dynamic ? Color.green : Color.yellow);
                    if (HighlightPenetration && _bodyPenDepth.TryGetValue(body, out float d) && d > PenIgnore)
                        col = PenColor(d); // 貫入色を優先
                    Gizmos.color = col;
                    DrawShape(body.Shape, MmdToUnityPos(body.WorldTransform.Origin), MmdToUnityRot(body.WorldTransform.Rotation));
                }

            // 接触点(球)と法線(線分, 長さ=貫入深さ)。
            if (DrawContacts)
                foreach (var pc in _penContacts)
                {
                    if (pc.depth <= PenIgnore) continue;
                    Gizmos.color = PenColor(pc.depth);
                    Gizmos.DrawSphere(pc.pos, 0.012f);
                    Gizmos.DrawLine(pc.pos, pc.pos + pc.normal * (pc.depth * ContactNormalScale * UnitScale));
                }

            DrawSceneLabel();

            // MMDゴースト: CSVのボーン姿勢 * オフセット で剛体位置を復元しマゼンタで重ね描き。
            if (DrawReferenceGhost && _csv != null && _simFrame >= 0)
            {
                Gizmos.color = Color.magenta;
                for (int i = 0; i < _builder.BoneLinks.Count; i++)
                {
                    var link = _builder.BoneLinks[i];
                    if (link.BoneIndex < 0 || link.BoneIndex >= _model.BoneNames.Count) continue;
                    string bn = _model.BoneNames[link.BoneIndex];
                    if (SkirtOnlyGhost && !bn.StartsWith("スカート")) continue;
                    if (!_csv.TryGet(_simFrame, bn, out var bw)) continue;
                    var refWorld = bw * link.BodyOffsetFromBone;
                    DrawShape(link.Body.Shape, MmdToUnityPos(refWorld.Origin), MmdToUnityRot(refWorld.Rotation));
                }
            }
        }

        private void DrawShape(CollisionShape shape, Vector3 pos, Quaternion rot)
        {
            var m = Gizmos.matrix;
            Gizmos.matrix = UnityEngine.Matrix4x4.TRS(pos, rot, Vector3.one * UnitScale);
            switch (shape)
            {
                case SphereShape s:
                    Gizmos.DrawWireSphere(Vector3.zero, s.Radius);
                    break;
                case BoxShape b:
                    Gizmos.DrawWireCube(Vector3.zero, new Vector3(b.HalfExtents.x, b.HalfExtents.y, b.HalfExtents.z) * 2f);
                    break;
                case CapsuleShape c:
                    Gizmos.DrawWireSphere(new Vector3(0, c.HalfHeight, 0), c.Radius);
                    Gizmos.DrawWireSphere(new Vector3(0, -c.HalfHeight, 0), c.Radius);
                    break;
            }
            Gizmos.matrix = m;
        }

        // B-4: Scene ビューに 現在フレーム / 最大貫入深さ / 該当剛体名 を表示 (Editor限定)。
        // UnityEditor 依存のため #if UNITY_EDITOR で囲む (ビルド/検証ハーネスからは除外される)。
        private void DrawSceneLabel()
        {
#if UNITY_EDITOR
            if (_maxPenA == "") return;
            var p = MmdToUnityPos(new Vec3(0, 0, 0));
            UnityEditor.Handles.color = _maxPenDepth >= PenDeep ? Color.red : Color.white;
            UnityEditor.Handles.Label(p,
                $"Frame {_simFrame}\n最大貫入 {_maxPenDepth:F3}\n{_maxPenA} × {_maxPenB}");
#endif
        }
    }
}
