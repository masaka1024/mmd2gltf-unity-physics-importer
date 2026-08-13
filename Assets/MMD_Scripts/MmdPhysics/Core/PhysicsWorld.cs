// ===========================================================================
// Unity Bullet 互換物理エンジン – PhysicsWorld
// btDiscreteDynamicsWorld 相当。重力・積分・衝突・Joint を統合する。
// Sequential-Impulse ソルバ + Baumgarte 位置補正。
// ===========================================================================

using System;
using System.Collections.Generic;

namespace BulletPhysics
{
    /// <summary>接触制約 (1 接触点 = 法線 + 2 摩擦)。</summary>
    internal struct ContactConstraint
    {
        public RigidBody A, B;
        public Vec3 RelA, RelB;
        public Vec3 Normal, Tangent1, Tangent2;
        public float NormalMass, TangentMass1, TangentMass2;
        public float NormalBias;       // 速度側の目標接近速度 (restitution + 浅貫入の Baumgarte)
        public float PushBias;         // Split Impulse: 擬似速度側の貫入回復目標 (深貫入のみ非0)
        public float Friction;
        public float NormalImpulse, TangentImpulse1, TangentImpulse2;
        public float PushImpulse;      // Split Impulse の蓄積擬似インパルス (ウォームスタートしない)
        public PersistentManifold Manifold; public int PointRef; // ウォームスタート書き戻し用
    }

    /// <summary>物理ワールド。剛体と Joint を保持しシミュレートする。</summary>
    public sealed class PhysicsWorld
    {
        public Vec3 Gravity = new(0f, -9.8f * 10f, 0f); // MMD スケール: 重力は約 98

        // ソルバ設定。
        // リファレンスは実効 1/60 (FixedTimeStep=1/30 は 30fps 入力に合わせ、SubSteps=2 で刻む)。
        // 刻み掃引で、実効刻みを細かくするほどMMDのスカート傾きに一致することを確認した
        // (12窓比 1/30:1.133 → 1/60:1.030 → 1/120:0.978)。外部実装も細刻み (Saba=1/120,
        // libmmd=1/60, MMDは物理最大60fps)。詳細は DESIGN.md「リファレンス刻み」節。
        // ※細刻み化は SubSteps で行う (FixedTimeStep を下げる経路はキネマティック補間の分母が
        //   フレーム総サブステップ数で正しく効く。両経路とも補間は修正済みだが、入力は 1/30 境界)。
        public int SolverIterations = 10;
        // ★2026-08-13: 2 → 4 (実効 1/60 → 1/120)。貫入対策。
        //   機構: 接触の検出帯は Collision.cs の SpeculativeMargin=0.02 という「速度を見ない固定距離」で、
        //   駆動剛体は接触点で 1/30 あたり中央 0.114 (法線成分 0.052) 動く = 帯の 5.7倍。
        //   接触が生成された時点で既に深く刺さっており、貫入オンセットの 58% は
        //   「前フレームに接触点ゼロ」から始まっていた。刻みを半分にすると 1ステップの移動量が減り、
        //   ソルバが押し出す機会も増えるので両方に効く。
        //   実測(5モデル): 貫入中央 -45〜-63%、悪化したモデルは無し。深貫入>0.5 は IA で 5件→0件、
        //   hairfid 7001フレームで 20件→0件。
        //   ★忠実度も同時に改善する: 12窓比 中央 1.0608 → 0.9867 (過去最良)、
        //   スカート p90 22.16 → 25.98 (MMD 25.66)。8 は行き過ぎ (傾き14.30/12窓比1.181)。
        //   ※過去に「SubSteps=4 は 12窓比 1.288 で不採用」と記録したが、あれはジョイント
        //   warm-start が ON の時の測定。warm-start はサブステップ間で蓄積を引き継ぐため
        //   細刻みほど悪化していた。撤去後は細刻みが素直に効く (両対策が噛み合っている)。
        //   コスト: 物理位相の計装合計 0.473 → 0.876 ms/step (約1.85倍, 30Hz予算の 4%→7%)。
        public int SubSteps = 4;
        public float FixedTimeStep = 1f / 30f;

        public float PenetrationSlop = 0.005f;
        public float BaumgarteFactor = 0.2f;
        public float RestitutionThreshold = 1.0f;

        // --- Split Impulse (接触の貫入回復を実速度から切り離す) ---
        // 貫入が閾値より深い接触のみ擬似速度側で回復し、実速度にエネルギーを注入しない。
        // 反発(restitution)は貫入回復ではないため実速度側に残す。ジョイントの ERP(Beta) は不変。
        // 既定は false (Bullet 2.75 の m_splitImpulse=false に準拠)。true で新方式へ切替。
        // ※効果検証の結果、過大スイング仮説は不支持 (反復依存が弱まらず一部窓は悪化) だったため、
        //   旧ベースライン維持と 2.75 準拠のため既定 false を選択。実装は比較用に温存する。
        public bool UseSplitImpulse = false;
        // ステップ2(b): ジョイントの Baumgarte 位置バイアスを split-impulse(擬似速度)へ分離する。
        // 接触用の UseSplitImpulse とは独立 (接触既定 false=Bullet2.75 準拠は不変)。既定 false=挙動不変。
        public bool UseJointSplitImpulse = false;
        // ステップ2(a-1): ジョイントの直線ロック行のみ warm-start (蓄積インパルスをサブステップ間で引継ぎ)。
        // 2026-08-09 に factor 0.85 で既定ON化した (VMD統計がMMDへ接近・12窓比が帯内・単鎖トルク3×改善)。
        // ★2026-08-12 既定OFFへ戻した。Bullet 2.75 は非接触拘束を warm-start しない
        //   (btSequentialImpulseConstraintSolver.cpp:788 で毎ステップ m_appliedImpulse=0)。
        //   0.85 でもラチェットは残っており、前髪チェーンが自励振動する。臨界係数はモデル依存で、
        //   中間値(0.70/0.50)は「どれかのモデルのピークを踏む」。5モデル実測で 0(=撤去) が唯一安全:
        //   待機区間の騒がしい3本 J中央 モデルA -89% / モデルN -41% / モデルB -32% (全モデルで全体最良)。
        //   代償はスカート平時傾き -0.46°(11.33→10.87, MMD 11.39)と髪×体貫入の1.1〜3.7倍増
        //   (ただし貫入>0.5 のフレームは全モデル0件)。12窓比はむしろ改善 1.0972→1.0608。
        //   A/B で旧既定に戻すときは両フラグを true にする (Joint.WarmStartFactor が 0.85 のまま残してある)。
        public bool UseJointWarmStart = false;
        // ステップ2(a-2): 角度行も warm-start (同一性キー=軸+側 lo/hi、同一性が変わればキャッシュ破棄)。
        // UseJointWarmStart と併用。既定OFF(上記と同じ理由)。
        public bool UseJointWarmStartAngular = false;
        public float SplitImpulsePenetrationThreshold = -0.02f; // Bullet 2.75 m_splitImpulsePenetrationThreshold

        // 求解順序: Bullet 2.75 solveSingleIteration は 1反復内で「ジョイント(NonContact)→接触→摩擦」の順に解き、
        // 接触がジョイントより後=接触が後勝ち。自前の従来は「接触→ジョイント」でジョイントが後勝ち(接触の押し出しが
        // 毎反復打ち消される=スカート貫入の主因)。ON で Bullet 同順(ジョイント→接触)に切替。
        // 既定 false=従来順(ビット不変)。ON=Bullet 準拠。
        public bool SolveJointsFirst = false;

        // 診断用: 直近の StepSimulation で実行された内部ステップ数 (0=蓄積のみ)。挙動に影響しない。
        public int LastStepsRun;

        // --- 位相別プロファイル (既定OFF=Stopwatch呼び出しゼロ=挙動/性能ともビット不変) ---
        // ON にすると SubStep 内の各位相の累積時間(ms)と回数を積む。最適化の効果測定用。
        public static bool ProfileEnabled = false;
        public static double ProfBroad, ProfBuild, ProfPrepare, ProfSpring, ProfWarm, ProfSolveContact, ProfSolveJoint, ProfIntegrate, ProfStore;
        public static long ProfSubSteps, ProfContacts, ProfManifolds;
        public static void ProfReset()
        {
            ProfBroad = ProfBuild = ProfPrepare = ProfSpring = ProfWarm = ProfSolveContact = ProfSolveJoint = ProfIntegrate = ProfStore = 0;
            ProfSubSteps = ProfContacts = ProfManifolds = 0;
        }
        private static readonly System.Diagnostics.Stopwatch _psw = new();
        private static double Tick() { _psw.Stop(); double ms = _psw.Elapsed.TotalMilliseconds; _psw.Restart(); return ms; }

        // 接触監査#5: Bullet は接触のwarm-startで蓄積インパルスに m_warmstartingFactor(0.85) を掛ける
        // (btContactSolverInfo.h:79)。当エンジンは従来 1.0 (減衰なし=残留を捨てない) だった。
        // ★2026-08-12 Bullet 準拠の 0.85 を既定化。モデルA 実測で髪の符号バイアス -2.74°→-0.52°
        //   (負=MMDより動かなさすぎ)、スカート平時傾き・深貫入はほぼ不変。
        //   ジョイント側 warm-start 撤去と併用したときの 12窓比は 1.0972→1.0608 で MMD へ接近する。
        public float ContactWarmStartFactor = 0.85f;

        // 接触監査#1+2: Bullet は 1反復内で法線→摩擦の順(摩擦は同反復の法線インパルスで上限決定)。
        // 当エンジンは従来 摩擦→法線(摩擦は前反復の法線を使用)。ON で Bullet 同順(法線先)。既定 false=ビット不変。
        public bool ContactNormalBeforeFriction = false;

        public readonly List<RigidBody> Bodies = new();
        public readonly List<Joint> Joints = new();

        private readonly Dictionary<long, PersistentManifold> _manifolds = new();
        private readonly List<ContactConstraint> _contacts = new();
        private readonly List<ContactPoint> _detectBuffer = new(2); // Detect の返り値受け取り
        private float _accumulator;

        public void AddBody(RigidBody b)
        {
            b.Index = Bodies.Count;
            // static/kinematic の目標姿勢を現在姿勢で初期化 (未設定でのテレポート防止)。
            // ボーン追従はこの後 MmdPhysicsBehaviour が毎フレーム上書きする。
            if (b.IsStaticOrKinematic)
                b.KinematicTarget = b.WorldTransform;
            Bodies.Add(b);
            _pairCount = -1; // 候補ペアを作り直す
        }

        public void AddJoint(Joint j) => Joints.Add(j);

        /// <summary>接触マニフォールド(蓄積インパルス含む)とアキュムレータをクリアする。
        /// 物理リセット時に前状態のウォームスタート値を持ち越さないために使う。</summary>
        public void ClearContacts()
        {
            _manifolds.Clear();
            _contacts.Clear();
            _accumulator = 0f;
        }

        private RigidTransform[] _frameKinStart; // フレーム開始時のキネマティック姿勢 (body index 単位)

        // --- 公開ステップ (可変 dt を固定ステップに分割) ---
        public void StepSimulation(float deltaTime)
        {
            _accumulator += deltaTime;

            // このフレームで走らせる内部ステップ数を先に確定する。
            // キネマティック(ボーン)目標の補間は「フレーム全体の総サブステップ数」を分母に等分する。
            // こうしないと、InternalStep を複数回呼ぶ場合に各内部ステップが個別に目標へ到達してしまい、
            // 最初の内部ステップで目標へジャンプ→残り停止、という誤補間になる (30fps入力を細分できない)。
            int stepsToRun = 0;
            { float rem = _accumulator; while (rem >= FixedTimeStep && stepsToRun < 8) { rem -= FixedTimeStep; stepsToRun++; } }
            LastStepsRun = stepsToRun; // 診断用 (タイミングログ)
            if (stepsToRun == 0) return;

            // フレーム開始時のキネマティック姿勢を保存。KinematicTarget はこのフレーム終端の姿勢。
            if (_frameKinStart == null || _frameKinStart.Length < Bodies.Count)
                _frameKinStart = new RigidTransform[Bodies.Count];
            for (int i = 0; i < Bodies.Count; i++)
                if (Bodies[i].IsKinematic) _frameKinStart[i] = Bodies[i].WorldTransform;

            int totalSub = stepsToRun * SubSteps; // フレーム内の総サブステップ数 (補間の分母)
            for (int k = 0; k < stepsToRun; k++)
            {
                InternalStep(FixedTimeStep, k * SubSteps, totalSub);
                _accumulator -= FixedTimeStep;
            }
            // 端数 (_accumulator の残り) は次回呼び出しへ持ち越す (標準的な固定刻みアキュムレータ)。
            // 入力は 30fps フレーム境界で更新されるため、KinematicTarget はこのフレーム終端で到達済みとして扱う。
        }

        // --- 固定 1 ステップ --- gBase: このフレーム内での先行サブステップ数, totalSub: フレーム総サブステップ数
        private void InternalStep(float dt, int gBase, int totalSub)
        {
            float sub = dt / SubSteps;
            for (int s = 0; s < SubSteps; s++)
                SubStep(sub, (float)(gBase + s + 1) / totalSub);
        }

        // frac: フレーム開始→終端目標の 等分補間割合 (1/totalSub .. 1)。フレーム全体で連続する。
        private void SubStep(float dt, float frac)
        {
            for (int i = 0; i < Bodies.Count; i++)
            {
                var b = Bodies[i];
                if (!b.IsKinematic) continue;
                b.KinematicStepTarget = InterpTransform(_frameKinStart[i], b.KinematicTarget, frac);
            }

            if (ProfileEnabled) { _psw.Restart(); }
            IntegrateVelocities(dt);
            if (ProfileEnabled) ProfIntegrate += Tick();
            BroadphaseNarrowphase();
            if (ProfileEnabled) { ProfBroad += Tick(); ProfManifolds += _manifolds.Count; }
            BuildContactConstraints(dt);
            if (ProfileEnabled) { ProfBuild += Tick(); ProfContacts += _contacts.Count; ProfSubSteps++; }

            foreach (var j in Joints) j.Prepare(dt, UseJointSplitImpulse, UseJointWarmStart, UseJointWarmStartAngular);
            if (ProfileEnabled) ProfPrepare += Tick();
            foreach (var j in Joints) j.ApplySprings(dt);
            if (ProfileEnabled) ProfSpring += Tick();

            WarmStart();
            if (ProfileEnabled) ProfWarm += Tick();

            for (int it = 0; it < SolverIterations; it++)
            {
                if (SolveJointsFirst)
                {
                    // Bullet 2.75 同順: ジョイント → 接触 (接触が後勝ち)。
                    foreach (var j in Joints) j.SolveVelocity();
                    if (ProfileEnabled) ProfSolveJoint += Tick();
                    SolveContacts();
                    if (ProfileEnabled) ProfSolveContact += Tick();
                }
                else
                {
                    // 従来順: 接触 → ジョイント (ジョイントが後勝ち)。
                    SolveContacts();
                    if (ProfileEnabled) ProfSolveContact += Tick();
                    foreach (var j in Joints) j.SolveVelocity();
                    if (ProfileEnabled) ProfSolveJoint += Tick();
                }
            }

            // Split Impulse: 実速度の求解後、貫入回復(接触)/位置補正(ジョイント)を擬似速度側で
            // 別反復して解く。擬似速度を 0 から解き、位置積分にのみ反映する (実速度には残さない)。
            if (UseSplitImpulse || UseJointSplitImpulse)
            {
                for (int i = 0; i < Bodies.Count; i++)
                {
                    Bodies[i].PseudoLinearVelocity = Vec3.Zero;
                    Bodies[i].PseudoAngularVelocity = Vec3.Zero;
                }
                for (int it = 0; it < SolverIterations; it++)
                {
                    if (UseSplitImpulse) SolveSplitImpulse();
                    if (UseJointSplitImpulse) foreach (var j in Joints) j.SolveSplitPosition();
                }
            }

            StoreImpulses();
            if (ProfileEnabled) ProfStore += Tick();
            IntegratePositions(dt);
            if (EnableSleeping) UpdateSleeping(dt);
        }

        // ═══════════════════════════════════════════
        //  スリープ (Bullet の deactivation 相当) — 2026-08-10 実装
        //
        //  症状: ほとんど静止しているのに揺れ物が細かく震え続ける。MMD(Bullet)は静止した
        //  剛体を非活性化して計算から外すので完全に止まる (ユーザー実機でMMDのIAの序盤=静止
        //  ポーズ中は髪の揺れが止まることを確認済み)。当エンジンは RigidBody に IsActive /
        //  SleepTimer の宣言だけがあり、どこからも使われていなかった。
        //
        //  Bullet と同じく「アイランド単位」で判定する。連結した剛体群の全員が眠りたがって
        //  いるときだけ、まとめて眠らせる。1体だけ眠らせると鎖の途中が固まって不自然になる。
        //  アイランドは「動的剛体どうしを繋ぐ Joint と接触」で連結する (Bullet 同様、
        //  static/kinematic はアイランドを繋がない = 体を介して髪とスカートが一体化しない)。
        //
        //  ★起こす条件が最重要。ここを誤ると「髪が固まって二度と動かない」というジッタより
        //    はるかに重い不具合になる。動いている kinematic (ボーン追従) 剛体に Joint or 接触で
        //    触れているアイランドは、眠りたがっていても眠らせない。ダンス中は体のボーンが
        //    動き続けるので、髪もスカートも常に起きたままになる。
        // ═══════════════════════════════════════════
        //  ★既定 OFF (2026-08-10)。実装はしたが現状ほとんど発動しない: 当エンジンの静止時の
        //    残留運動が Bullet のしきい値を超えているため (IA で |w|平均 1.5 > しきい値 1.0)。
        //    101体中 2体しか眠らず、効果が無い一方で「起こし損ねると固まる」リスクだけが残る。
        //    残留運動そのものを下げる方が先。下げられたら既定ONを検討する。
        public bool EnableSleeping = false;
        public float LinearSleepThreshold = 0.8f;    // Bullet 既定
        public float AngularSleepThreshold = 1.0f;   // Bullet 既定
        public float DeactivationTime = 2.0f;        // Bullet 既定 (秒)
        /// <summary>kinematic 剛体を「動いている」とみなす速度のしきい値。
        /// 完全に 0 でないと止まらない、を避けるための微小値。</summary>
        public float KinematicMotionEpsilon = 1e-4f;
        /// <summary>診断: 現在眠っている動的剛体の数。</summary>
        public int SleepingBodyCount { get; private set; }

        private int[] _uf;              // union-find の親 (剛体 index)
        private bool[] _wantsSleep;     // その剛体が眠りたがっているか
        private bool[] _islandBlocked;  // その根のアイランドは眠らせない
        private bool[] _islandWants;    // その根のアイランドは全員が眠りたがっている

        private int UfFind(int x) { while (_uf[x] != x) { _uf[x] = _uf[_uf[x]]; x = _uf[x]; } return x; }
        private void UfUnion(int a, int b) { a = UfFind(a); b = UfFind(b); if (a != b) _uf[a] = b; }

        private void UpdateSleeping(float dt)
        {
            int n = Bodies.Count;
            if (_uf == null || _uf.Length < n)
            { _uf = new int[n]; _wantsSleep = new bool[n]; _islandBlocked = new bool[n]; _islandWants = new bool[n]; }

            float lt2 = LinearSleepThreshold * LinearSleepThreshold;
            float at2 = AngularSleepThreshold * AngularSleepThreshold;

            // 1) 各動的剛体のタイマー更新。しきい値を超えていたら即リセット+起床。
            for (int i = 0; i < n; i++)
            {
                var b = Bodies[i];
                _uf[i] = i; _islandBlocked[i] = false; _islandWants[i] = true;
                _wantsSleep[i] = false;
                if (b.IsStaticOrKinematic) continue;
                if (b.LinearVelocity.LengthSquared < lt2 && b.AngularVelocity.LengthSquared < at2)
                    b.SleepTimer += dt;
                else { b.SleepTimer = 0f; b.IsActive = true; }
                _wantsSleep[i] = b.SleepTimer > DeactivationTime;
            }

            // 2) アイランド構築。動的どうしのみ連結する。
            foreach (var j in Joints)
            {
                if (j.BodyA == null || j.BodyB == null) continue;
                if (j.BodyA.IsStaticOrKinematic || j.BodyB.IsStaticOrKinematic) continue;
                UfUnion(j.BodyA.Index, j.BodyB.Index);
            }
            for (int c = 0; c < _contacts.Count; c++)
            {
                var a = _contacts[c].A; var b2 = _contacts[c].B;
                if (a == null || b2 == null) continue;
                if (a.IsStaticOrKinematic || b2.IsStaticOrKinematic) continue;
                UfUnion(a.Index, b2.Index);
            }

            // 3) 「動いている kinematic に触れているか」でアイランドを起床固定する。
            bool KinMoving(RigidBody k) =>
                k.LinearVelocity.LengthSquared > KinematicMotionEpsilon * KinematicMotionEpsilon ||
                k.AngularVelocity.LengthSquared > KinematicMotionEpsilon * KinematicMotionEpsilon;

            foreach (var j in Joints)
            {
                if (j.BodyA == null || j.BodyB == null) continue;
                if (j.BodyA.IsStaticOrKinematic && !j.BodyB.IsStaticOrKinematic)
                { if (KinMoving(j.BodyA)) _islandBlocked[UfFind(j.BodyB.Index)] = true; }
                else if (j.BodyB.IsStaticOrKinematic && !j.BodyA.IsStaticOrKinematic)
                { if (KinMoving(j.BodyB)) _islandBlocked[UfFind(j.BodyA.Index)] = true; }
            }
            for (int c = 0; c < _contacts.Count; c++)
            {
                var a = _contacts[c].A; var b2 = _contacts[c].B;
                if (a == null || b2 == null) continue;
                if (a.IsStaticOrKinematic && !b2.IsStaticOrKinematic)
                { if (KinMoving(a)) _islandBlocked[UfFind(b2.Index)] = true; }
                else if (b2.IsStaticOrKinematic && !a.IsStaticOrKinematic)
                { if (KinMoving(b2)) _islandBlocked[UfFind(a.Index)] = true; }
            }

            // 4) アイランド全員が眠りたがっているかを集約。
            for (int i = 0; i < n; i++)
            {
                if (Bodies[i].IsStaticOrKinematic) continue;
                if (!_wantsSleep[i]) _islandWants[UfFind(i)] = false;
            }

            // 5) 適用。眠るアイランドは速度を落として非活性化、それ以外は起こす。
            int sleeping = 0;
            for (int i = 0; i < n; i++)
            {
                var b = Bodies[i];
                if (b.IsStaticOrKinematic) continue;
                int r = UfFind(i);
                bool sleep = _islandWants[r] && !_islandBlocked[r];
                if (sleep)
                {
                    b.IsActive = false;
                    b.LinearVelocity = Vec3.Zero;
                    b.AngularVelocity = Vec3.Zero;
                    sleeping++;
                }
                else if (!b.IsActive)
                {
                    b.IsActive = true;
                    b.SleepTimer = 0f;   // 起きたらタイマーをやり直す
                }
            }
            SleepingBodyCount = sleeping;
        }

        // --- 速度積分: 重力/力/減衰 ---
        private void IntegrateVelocities(float dt)
        {
            foreach (var b in Bodies)
            {
                if (b.IsStaticOrKinematic)
                {
                    // Kinematic: 目標姿勢へ移動する速度を算出 (接触応答に使用)。
                    if (b.IsKinematic)
                    {
                        // このサブステップの補間目標への差分から速度を算出。
                        var cur = b.WorldTransform;
                        var tgt = b.KinematicStepTarget;
                        b.LinearVelocity = (tgt.Origin - cur.Origin) / dt;
                        var dq = tgt.Rotation * cur.Rotation.Conjugated();
                        b.AngularVelocity = QuatToAngularVelocity(dq, dt);
                    }
                    continue;
                }

                // 眠っている剛体は積分しない。重力を入れると毎ステップしきい値を超えて即起床し、
                // スリープが永久に成立しなくなる (Bullet も非活性化した剛体は積分対象外)。
                if (EnableSleeping && !b.IsActive) { b.ClearForces(); continue; }

                b.LinearVelocity += (Gravity + b.TotalForce * b.InverseMass) * dt;
                b.AngularVelocity += (b.InverseInertiaWorld * b.TotalTorque) * dt;

                // Bullet 2.75 btRigidBody::applyDamping と同じ秒単位の減衰。
                b.LinearVelocity *= DampingFactor(b.LinearDamping, dt);
                b.AngularVelocity *= DampingFactor(b.AngularDamping, dt);

                b.ClearForces();
            }
        }

        // Bullet 2.75 の秒単位減衰係数。(1 - d)^dt。
        // d=1.0 は 0 除算・完全停止を避けるためクランプする。
        private static float DampingFactor(float damping, float dt)
        {
            float d = Math.Clamp(damping, 0f, 0.999f);
            return (float)Math.Pow(1f - d, dt);
        }

        // --- 位置積分 ---
        private void IntegratePositions(float dt)
        {
            foreach (var b in Bodies)
            {
                if (b.IsStaticOrKinematic)
                {
                    if (b.IsKinematic)
                    {
                        b.WorldTransform = b.KinematicStepTarget;
                        b.UpdateInertiaWorld();
                    }
                    continue;
                }

                if (EnableSleeping && !b.IsActive) continue;   // 眠っている剛体は動かさない

                var t = b.WorldTransform;

                // Split Impulse 有効時は「実速度 + 擬似速度」で位置を進める (擬似は速度として残さない)。
                var vlin = b.LinearVelocity;
                var vang = b.AngularVelocity;
                if (UseSplitImpulse || UseJointSplitImpulse)
                {
                    vlin += b.PseudoLinearVelocity;
                    vang += b.PseudoAngularVelocity;
                }
                t.Origin += vlin * dt;

                // クォータニオン積分: q += 0.5 * w * q * dt。
                var w = vang;
                var spin = new Quat(w.x, w.y, w.z, 0f) * t.Rotation;
                t.Rotation = new Quat(
                    t.Rotation.x + spin.x * 0.5f * dt,
                    t.Rotation.y + spin.y * 0.5f * dt,
                    t.Rotation.z + spin.z * 0.5f * dt,
                    t.Rotation.w + spin.w * 0.5f * dt).Normalized;

                b.WorldTransform = t;
                b.UpdateInertiaWorld();
            }
        }

        private static Vec3 QuatToAngularVelocity(Quat dq, float dt)
        {
            dq = dq.Normalized;
            float angle = 2f * (float)Math.Acos(Math.Clamp(dq.w, -1f, 1f));
            if (angle < 1e-6f) return Vec3.Zero;
            if (angle > Math.PI) angle -= 2f * (float)Math.PI;
            var axis = new Vec3(dq.x, dq.y, dq.z);
            var len = axis.Length;
            if (len < 1e-9f) return Vec3.Zero;
            return axis / len * (angle / dt);
        }

        // 開始→目標を frac で補間 (位置は線形、回転は slerp)。
        private static RigidTransform InterpTransform(RigidTransform from, RigidTransform to, float frac)
        {
            return new RigidTransform(
                Quat.Slerp(from.Rotation, to.Rotation, frac),
                from.Origin + (to.Origin - from.Origin) * frac);
        }

        // --- ブロードフェーズ + ナローフェーズ ---
        // --- ブロードフェーズの候補ペア (最適化, 2026-08-09) ---
        // static/kinematic 同士の除外と ShouldCollide(Group/Mask) は「不変な情報」なので毎サブステップ
        // 総当たりで再判定する必要がない。初回に候補ペアを作り置きし、以後はそれだけを走査する。
        // (IA: 総当たり6786 → 候補のみへ削減。ペア順序は i昇順→k昇順 で従来と同一=結果ビット不変)
        // 剛体の追加や Group/CollisionMask/Mode を実行時に変更した場合は InvalidateCollisionPairs() を呼ぶこと
        // (AddBody は自動で無効化する。ハーネスは構築直後に変更するため初回構築前に確定する)。
        private int[] _pairA, _pairB; private int _pairCount = -1; private int _pairBuiltForCount = -1;
        private Aabb[] _aabbScratch; private readonly HashSet<long> _seenScratch = new(); private readonly List<long> _deadScratch = new();

        // ★このキャッシュが依存している前提 (崩れると「本来当たるペアが当たらない」形で静かに壊れる):
        //   1. Body の Group / CollisionMask が実行中に変わらない
        //   2. Body の Mode (static/kinematic/dynamic の別) が実行中に変わらない
        //   3. Bodies の増減は AddBody 経由 (自動で無効化する)
        // 現状の運用では全て成立するが、将来インパルスモーフ配線・剛体の動的追加/削除・
        // 実行時のモード切替を入れる場合は必ず InvalidateCollisionPairs() を呼ぶこと。
        // 壊れ方が静か(例外も警告も出ず、単に衝突しなくなる)なので、疑わしいときは
        // DebugCollisionPairCount を before/after で比較すること。

        /// <summary>Group/CollisionMask/Mode を実行時に変えたら呼ぶ (次ステップで候補ペアを作り直す)。
        /// 剛体の追加は AddBody が自動で無効化する。削除APIを足す場合も必ずここを呼ぶこと。</summary>
        public void InvalidateCollisionPairs() { _pairCount = -1; }

        /// <summary>診断/テスト用: 現在の候補ペア数 (未構築なら構築する)。挙動には影響しない。</summary>
        public int DebugCollisionPairCount
        {
            get { if (_pairCount < 0 || _pairBuiltForCount != Bodies.Count) BuildCollisionPairs(); return _pairCount; }
        }

        private void BuildCollisionPairs()
        {
            int n = Bodies.Count;
            int cap = n * (n - 1) / 2;
            if (_pairA == null || _pairA.Length < cap) { _pairA = new int[cap]; _pairB = new int[cap]; }
            int c = 0;
            for (int i = 0; i < n; i++)
                for (int k = i + 1; k < n; k++)
                {
                    var a = Bodies[i]; var b = Bodies[k];
                    if (a.IsStaticOrKinematic && b.IsStaticOrKinematic) continue;
                    if (!ShouldCollide(a, b)) continue;
                    _pairA[c] = i; _pairB[c] = k; c++;
                }
            _pairCount = c; _pairBuiltForCount = n;
        }

        private void BroadphaseNarrowphase()
        {
            int n = Bodies.Count;
            if (_pairCount < 0 || _pairBuiltForCount != n) BuildCollisionPairs();
            if (_aabbScratch == null || _aabbScratch.Length < n) _aabbScratch = new Aabb[n];
            var aabbs = _aabbScratch;
            // 検出帯を既定より広げているときだけ AABB も同じだけ膨らませる。
            // 広げないと「形状は帯の中なのに AABB 段階で捨てられる」ため帯の拡大が効かない。
            // 既定 (SpeculativeMargin=0.02) では extra=0 なので従来と完全に同一 = ビット不変。
            float extra = GjkEpa.SpeculativeMargin - GjkEpa.SpeculativeMarginDefault;
            if (extra > 0f) for (int i = 0; i < n; i++) { aabbs[i] = Bodies[i].ComputeAabb(); aabbs[i].Expand(extra); }
            else for (int i = 0; i < n; i++) aabbs[i] = Bodies[i].ComputeAabb();

            var seen = _seenScratch; seen.Clear();
            for (int p = 0; p < _pairCount; p++)
            {
                int i = _pairA[p], k = _pairB[p];
                if (!aabbs[i].Intersects(ref aabbs[k])) continue;
                var a = Bodies[i]; var b = Bodies[k];

                long key = PairKey(a.Index, b.Index);
                seen.Add(key);
                if (!_manifolds.TryGetValue(key, out var m))
                {
                    m = new PersistentManifold(a, b);
                    _manifolds[key] = m;
                }
                m.Refresh();
                _detectBuffer.Clear();
                GjkEpa.Detect(a, b, _detectBuffer);
                for (int di = 0; di < _detectBuffer.Count; di++)
                    m.AddPoint(_detectBuffer[di]);
            }
            // 消えたペアを掃除。
            if (_manifolds.Count > seen.Count)
            {
                var dead = _deadScratch; dead.Clear();
                foreach (var kv in _manifolds) if (!seen.Contains(kv.Key)) dead.Add(kv.Key);
                for (int d = 0; d < dead.Count; d++) _manifolds.Remove(dead[d]);
            }
        }

        /// <summary>
        /// PMX 衝突フィルタ。16bitフィールドは「衝突する相手グループ」のビットマスク
        /// (bit=1 で衝突する)。Bullet の (groupA & maskB) && (groupB & maskA) と同じ。
        /// </summary>
        public static bool ShouldCollide(RigidBody a, RigidBody b)
        {
            return (b.CollisionMask & (1 << a.Group)) != 0
                && (a.CollisionMask & (1 << b.Group)) != 0;
        }

        private static long PairKey(int a, int b)
        {
            if (a > b) (a, b) = (b, a);
            return ((long)a << 32) | (uint)b;
        }

        public int DebugContactCount; // 診断用

        // マニフォールドを決定的順序で解くための再利用バッファ (毎ステップのアロケーションを避ける)。
        private readonly List<PersistentManifold> _sortedManifolds = new();
        private static int CompareManifold(PersistentManifold x, PersistentManifold y) =>
            PairKey(x.BodyA.Index, x.BodyB.Index).CompareTo(PairKey(y.BodyA.Index, y.BodyB.Index));

        // --- 接触制約構築 ---
        private void BuildContactConstraints(float dt)
        {
            _contacts.Clear();

            // Dictionary の列挙順は挿入/削除履歴に依存し非決定的なので、剛体indexの組(PairKey)の
            // 昇順に並べ替えてから制約を構築する。これにより「無関係な剛体の増減」で接触の解く順序が
            // 変わって結果が揺れる (Gauss-Seidel の順序依存) 現象を排除する。式・パラメータは不変、順序のみ。
            _sortedManifolds.Clear();
            foreach (var kv in _manifolds) _sortedManifolds.Add(kv.Value);
            _sortedManifolds.Sort(CompareManifold);

            foreach (var m in _sortedManifolds)
            {
                for (int p = 0; p < m.Points.Count; p++)
                {
                    var cp = m.Points[p];
                    var a = m.BodyA; var b = m.BodyB;

                    var rA = cp.PositionWorldA - a.CenterOfMass;
                    var rB = cp.PositionWorldB - b.CenterOfMass;
                    // EPA 法線は A→B 方向。ソルバ規約 (relVel=vB-vA, -P:A/+P:B) と一致。
                    var n = cp.Normal;

                    // 接線基底。
                    BuildTangentBasis(n, out var t1, out var t2);

                    var cc = new ContactConstraint
                    {
                        A = a, B = b, RelA = rA, RelB = rB,
                        Normal = n, Tangent1 = t1, Tangent2 = t2,
                        Friction = (float)Math.Sqrt(Math.Max(0, a.Friction) * Math.Max(0, b.Friction)),
                        NormalMass = EffectiveMass(a, b, rA, rB, n),
                        TangentMass1 = EffectiveMass(a, b, rA, rB, t1),
                        TangentMass2 = EffectiveMass(a, b, rA, rB, t2),
                        NormalImpulse = cp.NormalImpulse,
                        TangentImpulse1 = cp.TangentImpulse1,
                        TangentImpulse2 = cp.TangentImpulse2,
                        Manifold = m, PointRef = p,
                    };

                    // 法線の目標接近速度 (NormalBias) を決める。
                    var relN = (b.VelocityAtPoint(cp.PositionWorldB) - a.VelocityAtPoint(cp.PositionWorldA)).Dot(n);
                    float rest = (float)Math.Sqrt(Math.Max(0, a.Restitution) * Math.Max(0, b.Restitution));
                    float restBias = (-relN > RestitutionThreshold) ? rest * -relN : 0f;

                    if (cp.Distance <= 0f)
                    {
                        // 貫入: Baumgarte 位置補正 + 反発。
                        float pen = -cp.Distance - PenetrationSlop;
                        float biasVel = pen > 0 ? BaumgarteFactor * pen / dt : 0f;
                        // Split Impulse: Bullet 2.75 convertContact と同様、貫入が閾値より深い接触のみ
                        // 位置補正を擬似速度側(PushBias)へ回し、実速度側は反発のみにする。
                        // 浅い貫入 (閾値より浅い) は従来どおり速度側の Baumgarte に載せる。
                        if (UseSplitImpulse && cp.Distance <= SplitImpulsePenetrationThreshold)
                        {
                            cc.NormalBias = restBias;   // 実速度側: 反発のみ (貫入回復は載せない)
                            cc.PushBias = biasVel;      // 擬似速度側: 貫入回復
                        }
                        else
                        {
                            cc.NormalBias = Math.Max(biasVel, restBias); // 従来どおり
                            cc.PushBias = 0f;
                        }
                    }
                    else
                    {
                        // 非貫入 (投機的接触): このステップで表面へちょうど到達する接近
                        // (-Distance/dt) までは許し、それを超える接近だけを止める。押し戻さない。
                        // 反発は押し戻す向きなので、より接近を許す (小さい) 方を採る。
                        float speculative = -cp.Distance / dt;
                        cc.NormalBias = Math.Min(speculative, restBias);
                    }
                    _contacts.Add(cc);
                }
            }
            DebugContactCount = _contacts.Count;
        }

        private static float EffectiveMass(RigidBody a, RigidBody b, Vec3 rA, Vec3 rB, Vec3 dir)
        {
            var rAxn = Vec3.Cross(rA, dir);
            var rBxn = Vec3.Cross(rB, dir);
            float k = a.InverseMass + b.InverseMass
                    + rAxn.Dot(a.InverseInertiaWorld * rAxn)
                    + rBxn.Dot(b.InverseInertiaWorld * rBxn);
            return k > 0 ? 1f / k : 0f;
        }

        private static void BuildTangentBasis(Vec3 n, out Vec3 t1, out Vec3 t2)
        {
            if (Math.Abs(n.x) >= 0.577f)
                t1 = new Vec3(n.y, -n.x, 0f).Normalized;
            else
                t1 = new Vec3(0f, n.z, -n.y).Normalized;
            t2 = Vec3.Cross(n, t1);
        }

        // 検証用の読み取り専用診断フック。null (既定) の間は何もせず、挙動・性能に影響しない。
        // 回帰テスト (非貫入押し出しの検出など) が接触の Distance/法線インパルスを参照するために使う。
        public System.Collections.Generic.List<(string a, string b, float dist, float ni)> DebugContacts;

        // 蓄積インパルスを manifold へ書き戻し、次フレームのウォームスタートに使う。
        private void StoreImpulses()
        {
            for (int i = 0; i < _contacts.Count; i++)
            {
                var c = _contacts[i];
                if (c.Manifold == null || c.PointRef >= c.Manifold.Points.Count) continue;
                var cp = c.Manifold.Points[c.PointRef];
                cp.NormalImpulse = c.NormalImpulse;
                cp.TangentImpulse1 = c.TangentImpulse1;
                cp.TangentImpulse2 = c.TangentImpulse2;
                c.Manifold.Points[c.PointRef] = cp;
                DebugContacts?.Add((c.A.Name, c.B.Name, cp.Distance, c.NormalImpulse));
            }
        }

        private void WarmStart()
        {
            float wf = ContactWarmStartFactor;
            for (int i = 0; i < _contacts.Count; i++)
            {
                var c = _contacts[i];
                // Bullet同様、蓄積インパルスに係数を掛けてから適用+アキュムレータ初期値にする(0.85時)。
                if (wf != 1.0f) { c.NormalImpulse *= wf; c.TangentImpulse1 *= wf; c.TangentImpulse2 *= wf; }
                var P = c.Normal * c.NormalImpulse
                      + c.Tangent1 * c.TangentImpulse1
                      + c.Tangent2 * c.TangentImpulse2;
                c.A.ApplyImpulse(-P, c.RelA);
                c.B.ApplyImpulse(P, c.RelB);
                _contacts[i] = c;
            }
        }

        // --- 接触速度求解 ---
        private void SolveContacts()
        {
            for (int i = 0; i < _contacts.Count; i++)
            {
                var c = _contacts[i];
                var a = c.A; var b = c.B;

                if (!ContactNormalBeforeFriction)
                {
                    // 従来: 摩擦(前反復の法線で上限) → 法線。
                    SolveFriction(ref c, a, b, c.Tangent1, c.TangentMass1, ref c.TangentImpulse1);
                    SolveFriction(ref c, a, b, c.Tangent2, c.TangentMass2, ref c.TangentImpulse2);
                    SolveNormal(ref c, a, b);
                }
                else
                {
                    // Bullet同順: 法線 → 摩擦(同反復の法線で上限)。
                    SolveNormal(ref c, a, b);
                    SolveFriction(ref c, a, b, c.Tangent1, c.TangentMass1, ref c.TangentImpulse1);
                    SolveFriction(ref c, a, b, c.Tangent2, c.TangentMass2, ref c.TangentImpulse2);
                }

                _contacts[i] = c;
            }
        }

        // --- Split Impulse 求解 (擬似速度で貫入回復) ---
        // SolveContacts の法線部と同一の符号規約。ただし擬似速度に対して解き、目標は PushBias。
        // 蓄積擬似インパルス(PushImpulse)は 0 以上にクランプ。ウォームスタートしない。
        private void SolveSplitImpulse()
        {
            for (int i = 0; i < _contacts.Count; i++)
            {
                var c = _contacts[i];
                if (c.PushBias <= 0f) continue; // 深い貫入の接触のみ
                var a = c.A; var b = c.B;
                var pA = a.CenterOfMass + c.RelA;
                var pB = b.CenterOfMass + c.RelB;
                float relN = (b.PseudoVelocityAtPoint(pB) - a.PseudoVelocityAtPoint(pA)).Dot(c.Normal);
                float dP = (c.PushBias - relN) * c.NormalMass;
                float old = c.PushImpulse;
                c.PushImpulse = Math.Max(0f, old + dP);
                dP = c.PushImpulse - old;
                var P = c.Normal * dP;
                a.ApplyPushImpulse(-P, c.RelA);
                b.ApplyPushImpulse(P, c.RelB);
                _contacts[i] = c;
            }
        }

        private static void SolveNormal(ref ContactConstraint c, RigidBody a, RigidBody b)
        {
            var pA = a.CenterOfMass + c.RelA;
            var pB = b.CenterOfMass + c.RelB;
            float relN = (b.VelocityAtPoint(pB) - a.VelocityAtPoint(pA)).Dot(c.Normal);
            float dPn = (c.NormalBias - relN) * c.NormalMass;
            float oldN = c.NormalImpulse;
            c.NormalImpulse = Math.Max(0f, oldN + dPn);
            dPn = c.NormalImpulse - oldN;
            var Pn = c.Normal * dPn;
            a.ApplyImpulse(-Pn, c.RelA);
            b.ApplyImpulse(Pn, c.RelB);
        }

        private static void SolveFriction(ref ContactConstraint c, RigidBody a, RigidBody b,
            Vec3 tangent, float mass, ref float accum)
        {
            var pA = a.CenterOfMass + c.RelA;
            var pB = b.CenterOfMass + c.RelB;
            float relT = (b.VelocityAtPoint(pB) - a.VelocityAtPoint(pA)).Dot(tangent);
            float dPt = -relT * mass;
            float maxF = c.Friction * c.NormalImpulse;
            float old = accum;
            accum = Math.Max(-maxF, Math.Min(maxF, old + dPt));
            dPt = accum - old;
            var Pt = tangent * dPt;
            a.ApplyImpulse(-Pt, c.RelA);
            b.ApplyImpulse(Pt, c.RelB);
        }
    }
}
