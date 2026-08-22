// ===========================================================================
// Unity Bullet 互換物理エンジン – Joint Constraints
// PMX Joint 6 種を Bullet 相当の Sequential-Impulse で解く。
//   0: ﾊﾞﾈ付6DOF -> btGeneric6DofSpringConstraint
//   1: 6DOF      -> btGeneric6DofConstraint
//   2: P2P       -> btPoint2PointConstraint
//   3: ConeTwist -> btConeTwistConstraint
//   4: Slider    -> btSliderConstraint
//   5: Hinge     -> btHingeConstraint
// ===========================================================================

using System;
using System.Collections.Generic;

namespace BulletPhysics
{
    public enum JointType
    {
        Spring6Dof = 0,
        Generic6Dof = 1,
        Point2Point = 2,
        ConeTwist = 3,
        Slider = 4,
        Hinge = 5,
    }

    /// <summary>制約ソルバの 1 行 (linear または angular)。</summary>
    internal struct ConstraintRow
    {
        public Vec3 Axis;         // ワールド軸 (単位)
        public bool Angular;      // true=回転行, false=並進行
        public Vec3 RelA, RelB;   // 重心からのアンカーオフセット (linear 用)
        public float LowerImpulse, UpperImpulse;
        public float TargetVel;   // 目標相対速度 (split時=バイアス抜きの速度目標, 非split時=Baumgarte込み)
        public float EffMass;
        public float Accumulated;
        public float PositionBias;    // split時: 位置補正の目標擬似速度 (err*Beta/dt)。非split時0。
        public float PseudoAccumulated; // split時: 擬似速度側の累積インパルス。
        public int Dof;               // 0-2: DOF番号 (warm-start のキャッシュキー)。
        public bool WarmStartable;    // (a-1) この行を warm-start 対象にするか (直線ロック行のみ)。
    }

    /// <summary>
    /// 全 Joint 種の基盤となる 6DOF 制約。
    /// PMX の位置/回転/移動制限/回転制限/バネ定数をそのまま解釈する。
    /// </summary>
    public sealed class Joint
    {
        public string Name = string.Empty;
        public JointType Type = JointType.Spring6Dof;

        public RigidBody BodyA;
        public RigidBody BodyB;

        // 剛体ローカルに変換済みのジョイントフレーム。
        public RigidTransform FrameInA = RigidTransform.Identity;
        public RigidTransform FrameInB = RigidTransform.Identity;

        // 移動/回転の制限 (下限, 上限)。
        public Vec3 LinearLowerLimit;
        public Vec3 LinearUpperLimit;
        public Vec3 AngularLowerLimit;
        public Vec3 AngularUpperLimit;

        // バネ定数 (移動/回転)。0 で無効。
        public Vec3 SpringLinear;
        public Vec3 SpringAngular;

        // バネのダンピング比 (PMX には無いので既定値)。
        public float SpringDamping = 0.1f;

        // 位置補正係数 (Baumgarte)。
        public float Beta = 0.2f;
        /// <summary>A/B用: 角度リミット行だけ位置補正係数を倍率で変える。既定 1 = 変更なし
        /// (float の 1.0 倍は IEEE754 で厳密なのでビット不変)。0 で角度行の ERP チャンネルだけを切れる。
        /// 「角度行が err 再生成の犯人か」を並進行と分離して判定するための診断用。</summary>
        public static float AngularBetaScale = 1f;

        /// <summary>A/B (既定 0 = 無効・ビット不変): 誤差デッドバンド。
        /// |err| がこの値未満の補正行を **そもそも構築しない** (目標速度0の行を作るのではなく、行自体を作らない)。
        /// 狙い: 参照データの静区間には完全な凍結が無く、ボーン別中央 |Δp| のフロアが 0.0028 PMX単位/フレーム
        /// である (2026-08-21 実測) ため、それより細かい誤差を追い続ける補正が残留振動の源になっている。
        /// ★全域ダイヤルではない: 誤差が閾値以上の場面・モデルでは行が満額で立つので、補正を強く必要とする
        /// 構成 (鎖が伸びる・貫入が深い等) には一切触れない。触れるのは「誤差が既にフロア未満の行」だけ。
        /// 単位に注意: 並進行は PMX 単位、回転行はラジアン。両方に同じ数値が適用される。</summary>
        public static float ErrDeadband = 0f;

        /// <summary>A/B (既定 0 = 無効・ビット不変): **バイアスのみ**デッドバンド。**並進行のみ**に効く。
        /// |err| がこの値未満のとき、行は通常どおり構築したうえで **目標速度の位置補正成分だけを 0** にする。
        /// 速度拘束としての行は完全に生きるので、`ErrDeadband` (行ごと削除) で起きたバンバン発振
        /// — 抑制中に相対速度が自由になり、誤差が閾値を超えると叩き戻される — が原理的に起きない。
        /// 狙いは §「参照の静区間には凍結が無く、ボーン別中央 |Δp| のフロアが 0.0028 PMX単位/フレーム」
        /// より細かい誤差を追い続ける補正を止めること。
        /// ★全域ダイヤルではない: 誤差が閾値以上の行は満額でバイアスが立つので、補正を強く必要とする
        /// 構成 (鎖が伸びる・貫入が深い等) には一切触れない。</summary>
        public static float BiasDeadband = 0f;

        /// <summary>構造分類: 並進DOFに「自由(lo&gt;hi)」または「範囲(lo&lt;hi)」が1軸でもあれば横渡し型。
        /// 3軸すべてロック(lo==hi)なら鎖型。**名前ではなく limit 値で判定する。**
        /// 2026-08-21 の最小網スキャンで、鎖型だけなら53本でも静止時 |w| 中央が 0.00 なのに対し、
        /// 横渡し型を1本足すだけで 95.45 に跳ねることが分かったため、この2種を区別できるようにした。</summary>
        public bool IsCrossTypeJoint
        {
            get
            {
                for (int i = 0; i < 3; i++) if (LinearLowerLimit[i] != LinearUpperLimit[i]) return true;
                return false;
            }
        }

        /// <summary>A/B (既定 false = ビット不変): **横渡し型のジョイントだけ** 位置補正を
        /// split-impulse (擬似速度) 側へ回す。鎖型 (全軸ロック) は既定のまま実速度側に載せる。
        /// 全域の UseJointSplitImpulse は「中央比 1.002 だが p90比 0.429 = 尾が痩せる」ため不採用だったが、
        /// 残留の発生源は横渡し型に限局していると分かったので、適用範囲を構造で絞る。</summary>
        public static bool SplitCrossOnly = false;
        /// <summary>診断 (既定 false): 上の補集合。**鎖型 (全軸ロック) だけ** split する。
        /// 「JSPLIT 全域の効果が鎖型・横渡し型のどちらから来ているか」を分解するための対照。</summary>
        public static bool SplitChainOnly = false;

        /// <summary>診断専用 (既定 false = ビット不変): 横渡し型の並進ロック軸を**バインド時の world 方向へ凍結**する。
        /// 「ロック軸が剛体と一緒に掃くことで dt 非依存の誤差が湧く」という増幅器仮説の直接検証用。
        /// 運動中は軸が剛体に追従すべきなので**出荷候補ではない**。</summary>
        public static bool FreezeCrossAxes = false;
        private Vec3[] _bindAxes;   // 凍結軸 (最初の Prepare で採取)
        /// <summary>検証用の読み取り専用診断フック。null (既定) の間は何もせず、挙動・性能に影響しない。
        /// 角度3軸の毎サブステップの状態を出す。state: 0=free / 1=範囲内(行なし) / 2=locked / 3=下限外 / 4=上限外。
        /// 「静止しているのに拘束行が現れたり消えたりしていないか」(不等式拘束のチャタリング) を見るため。</summary>
        public static System.Collections.Generic.List<(string joint, int dof, int state, float cur, float err)> DebugAngularRows;

        /// <summary>検証用の読み取り専用診断フック。null (既定) の間は何もせず、挙動・性能に影響しない。
        /// dt スケーリング監査用: 補正行を作るたびに (ジョイント名, DOF, 回転行か, err, 目標速度=Clamp(err*Beta*invDt),
        /// 行構築時点の相対速度) を記録する。サブステップ毎に Prepare が呼ばれるので、1フレーム分を
        /// まとめて読めば「サブステップ何回ぶんの補正が注入されたか」を外から積算できる。
        /// `DebugAngularRows` / `DebugContacts` と同じ流儀 (既定 null・読み取りのみ)。</summary>
        public static System.Collections.Generic.List<(string joint, int dof, bool angular, float err, float targetVel, float relVel)> DebugRows;

        /// <summary>検証用の読み取り専用診断フック。null (既定) の間は何もしない。
        /// SolveVelocity の各行を解いた直後に、その行の軸・レバーアーム・累積インパルス・目標速度・
        /// 適用後の相対速度を記録する。反復ごとに積まれるので、末尾 (最終反復ぶん) を読むこと。
        /// `DebugRowsSolvedJoint` に名前を入れると、そのジョイント名を含む行だけに絞れる (データ量対策)。</summary>
        public static System.Collections.Generic.List<(string joint, int dof, bool angular,
            Vec3 axis, Vec3 relA, Vec3 relB, string bodyA, string bodyB,
            float accumulated, float targetVel, float relVelAfter)> DebugRowsSolved;
        /// <summary>DebugRowsSolved の絞り込み。null = 全ジョイント。部分一致。</summary>
        public static string DebugRowsSolvedJoint;

        // 位置補正速度の上限。Bullet の線形行には存在しない自前のみの安全弁 (監査#3)。
        // 既定 10 =従来値(ビット不変)。1e9 相当で実質無効=Bullet同等。static につき env から A/B 可。
        public static float MaxCorrectionVel = 10f;

        // 角度リミット行の軸: Bullet 混合軸 (calculateAngleInfo, 2026-08-09 実ソース取得済み)。
        //   axis0 = B基底列0, axis2 = A基底列2,
        //   axis[1]=axis2×axis0, axis[0]=axis[1]×axis2, axis[2]=axis0×axis[1] (各 normalize, 一般に非直交)。
        // 自前既定は A基底の直交列(_axesA)。8/07に一度試して破棄したが、当時は
        // (1)軸と誤差の対応順序が未確認のままで実装誤りの可能性 (2)判定相手が補正ON版のみ。
        // 補正OFF版(純Bullet)を相手に再評価するため復活。既定 false=従来(ビット不変)。
        public static bool AngularMixedAxes = false;

        /// <summary>A/B (既定 false = ビット不変): 角度の抽出規約を **Bullet 2.75 の実挙動** に合わせる。
        /// 2026-08-22 のタスク21 で、同一姿勢に対する相対角が 15 DOF すべて食い違うことが判明した。
        /// 外部の独立計算で両側の値を厳密に再現して確定した内訳:
        ///   当エンジン = XYZオイラー(R_A⁻¹ R_B)   /   Bullet 2.75 = XYZオイラー(R_B⁻¹ R_A)
        /// 抽出式そのものは両者同一 (y=asin(m02) / x=atan2(-m12,m22) / z=atan2(-m01,m00)) で、
        /// Bullet も calculateAngleInfo で R_A⁻¹R_B を渡している。それでも転置になるのは
        /// 要素アクセスが列優先だから:
        ///   btGeneric6DofConstraint.cpp:57-62
        ///     btScalar btGetMatrixElem(const btMatrix3x3&amp; mat, int index)
        ///     { int i = index%3; int j = index/3; return mat[i][j]; }   // index=2 は mat[2][0]=m20
        /// btMatrix3x3 は行優先なので matrixToEulerXYZ は**渡された行列の転置**に対して働く。
        /// ★単なる符号反転ではない: 逆回転の XYZ オイラーは元の負値と一致しないので**大きさが変わり**、
        ///   どのリミットを超えるかが変わる (実測: 0.3894 vs 0.2950 で、片方だけ ±20° 制限を割る)。
        ///
        /// 実装は2点セットで意味を持つ:
        ///   (1) cur = XYZオイラー(R_B⁻¹ R_A)
        ///   (2) **角度行の軸を反転**する。cur の時間微分が (wA-wB)·axis になるので、
        ///       相対速度を (wB-wA)·axis で測る当エンジンの行と符号を合わせるために要る。
        ///       これで err の作り方・不等式の側・力積の上下限が Bullet と1対1で対応する。
        /// 軸そのものの選び方 (A基底 vs 混合軸) は <see cref="AngularMixedAxes"/> の担当で、こことは独立。</summary>
        public static bool BulletAngleConvention = false;

        /// <summary>A/B (既定 false = ビット不変): **ばねを Bullet 2.75 と同じ「モーター行」として解く。**
        /// タスク32。当エンジンは陽的力積 (`ApplySprings` で `-k*err*dt` を直接 ApplyImpulse し
        /// デッドビートでクランプ) だが、Bullet 2.75 は
        /// `btGeneric6DofSpringConstraint::internalUpdateSprings` でばね力を**リミットモーターの
        /// 目標速度と最大力**に変換し、**ソルバ行として反復内で解く**。構造的に別物。
        /// 実ソース (2.75) の式そのまま:
        ///   linear : delta = currPos - eq;  force = delta * k
        ///   angular: delta = currPos - eq;  force = -delta * k
        ///   velFactor = fps * springDamping / numIterations     (springDamping の既定は **1.0**)
        ///   targetVelocity = velFactor * force ;  maxMotorForce = |force| / fps
        /// さらに `get_limit_motor_info2` は
        ///   - リミット違反中 (limit != 0) はモーター行を出さない
        ///   - ロック軸 (lo==hi) では powered = 0
        ///   - 目標速度に `getMotorFactor` を掛けてリミット手前で先細りさせる
        ///   - 力積の上下限は ±maxMotorForce
        /// ★当てるパラメータは作っていない。係数は全て Bullet 実ソースの値
        ///   (springDamping=1.0 / timeFact = fps*erp で erp は <see cref="Beta"/>)。
        /// ★ON の間は `ApplySprings` の陽的経路を通さない (二重適用を防ぐ)。両経路は併存させてある。
        ///
        /// ★★2026-08-22 (タスク37) 既定を **ON** にした。根拠:
        ///   - 行レベルで Bullet と等価と確認済み (ばね付き最小網 24行×10反復、
        ///     effMass 4.1e-07 / bias 2.7e-04 / 累積力積 2.75e-04 = float 丸めの範囲)
        ///   - 当てるパラメータをひとつも作っていない (係数は全て Bullet 実ソースの値)
        ///   - 採用ゲート全通過: bonecheck ビット不変 (IA はばね0) /
        ///     31モデル 中央 改善13・悪化5、拘束違反 改善10・悪化9、NaN 発散 0 /
        ///     最小網 ビット不変 / モデルR はクランプ非適用でも NaN なし
        ///   - 駆動ゲート (drivedp, モデルB) は7部位すべてで参照比が 1.00 へ寄る
        ///   - 静止ゲート (Tda スカート |Δp|) が参照比 8.43x → 3.84x</summary>
        public static bool SpringAsMotorRow = true;

        /// <summary>Bullet 2.75 `btGeneric6DofSpringConstraint` の `m_springDamping` 既定値。
        /// PMX にはこの値が無いので **Bullet の既定 1.0 をそのまま使う**。つまみにしない。</summary>
        public const float BulletSpringDamping = 1.0f;

        /// <summary>Bullet 2.75 `btTypedConstraint.cpp` の `getMotorFactor` をそのまま移植。
        /// リミットに近づくとモーターの目標速度を先細りさせる。</summary>
        internal static float MotorFactor(float pos, float lowLim, float uppLim, float vel, float timeFact)
        {
            if (lowLim > uppLim) return 1f;
            if (lowLim == uppLim) return 0f;
            float limFact = 1f;
            float deltaMax = timeFact != 0f ? vel / timeFact : 0f;
            if (deltaMax < 0f)
            {
                if (pos >= lowLim && pos < (lowLim - deltaMax)) limFact = (lowLim - pos) / deltaMax;
                else if (pos < lowLim) limFact = 0f;
                else limFact = 1f;
            }
            else if (deltaMax > 0f)
            {
                if (pos <= uppLim && pos > (uppLim - deltaMax)) limFact = (uppLim - pos) / deltaMax;
                else if (pos > uppLim) limFact = 0f;
                else limFact = 1f;
            }
            else limFact = 0f;
            return limFact;
        }

        // 線形ロック行のレバーアーム基準 (線形行監査#1, 2026-08-09)。既定 0=従来(ビット不変)。
        //  0=従来: rA=anchorA-comA, rB=anchorB-comB (各剛体が自分側のアンカーを使う)
        //  1=Bullet2.75系(非offset): 両剛体とも B側アンカー基準 ("Linear Torque Decoupling")
        //     J1ang=(anchorB-posA)×ax, J2ang=-(anchorB-posB)×ax → 誤差があると親へ e×P の結合トルクが伝わる
        //  2=Bullet2.8x系(offset, D6_USE_FRAME_OFFSET true 既定): 軸平行成分を除去した ortho +
        //     totalDist を質量比 factA=miB/(miA+miB) で分配。hasStaticBody&&!rotAllowed で fact スケール。
        public static int LinearLeverMode = 0;

        // 内部状態。
        private readonly List<ConstraintRow> _rows = new(6);
        private RigidTransform _worldA, _worldB;
        private Vec3 _anchorA, _anchorB;
        private Vec3[] _axesA = new Vec3[3];
        // ステップ2(b): Baumgarte バイアスを split-impulse(擬似速度)側へ分離するか。
        // false(既定)で従来どおり実速度側へバイアスを乗せる (挙動不変)。
        private bool _splitBias;
        // ステップ2(a-1): 直線ロック行のみ warm-start (蓄積インパルスをサブステップ間で引き継ぐ)。
        // 直線ロック行は常時アクティブで DOF が安定 (角度行のような軸/側のトグルが無い) ため安全。
        private bool _warmStart;
        private readonly float[] _warmLin = new float[3];   // 直線 DOF 別の前サブステップ蓄積インパルス。
        private readonly bool[] _warmLinSeen = new bool[3];  // このサブステップで当該 DOF の warm 対象行が出たか。
        // ステップ2(a-2): 角度 warm-start。同一性キー=軸(dof)+側(sideCode: 0=locked/1=lower/2=upper)。
        // 前サブステップと同じ側なら引き継ぎ、変われば破棄する (Euler 分解で軸/側がトグルする問題対策)。
        private bool _warmStartAng;
        private readonly float[] _warmAng = new float[3];        // 角度 DOF 別の蓄積インパルス。
        private readonly int[] _warmAngPrevSide = { -1, -1, -1 }; // 前サブステップの側 (-1=不在)。
        private readonly bool[] _warmAngSeen = new bool[3];       // このサブステップで当該 DOF の角度 warm 行が出たか。
        // 診断: 角度 warm 行の総数と、同一性(側)が前サブステップから変わった行数。
        public static long WarmAngRows, WarmAngToggles;
        // warm-start 引継ぎ係数。過拘束系で引継ぎインパルスがラチェット的に積み上がるのを抑える。
        // ★2026-08-12: ジョイントの warm-start 自体を既定OFFにしたので、この値は
        //   PhysicsWorld.UseJointWarmStart(Angular) を true に戻したときだけ効く (A/B用)。
        //   0.85 は Bullet の接触側 m_warmstartingFactor から借りた値で、旧既定の再現用に残してある。
        //   ★この値でジョイントを warm-start してはいけない: 0.85 でもラチェット共振が残り、
        //   臨界係数がモデル依存で動くため中間値はどれかのモデルで悪化する (PhysicsWorld.cs の注記参照)。
        public static float WarmStartFactor = 0.85f;

        // --- ファクトリ: PMX Joint 種から生成 ---

        /// <summary>PMX の raw パラメータから Joint を構築する。</summary>
        public static Joint FromPmx(
            JointType type, RigidBody a, RigidBody b,
            RigidTransform worldFrame,
            Vec3 linLo, Vec3 linHi, Vec3 angLo, Vec3 angHi,
            Vec3 springLin, Vec3 springAng)
        {
            var j = new Joint
            {
                Type = type,
                BodyA = a,
                BodyB = b,
                LinearLowerLimit = linLo,
                LinearUpperLimit = linHi,
                AngularLowerLimit = angLo,
                AngularUpperLimit = angHi,
                SpringLinear = springLin,
                SpringAngular = springAng,
            };

            // フレームを各剛体ローカルへ落とし込む。
            j.FrameInA = a != null ? a.WorldTransform.InverseTimes(worldFrame) : worldFrame;
            j.FrameInB = b != null ? b.WorldTransform.InverseTimes(worldFrame) : worldFrame;

            // Joint 種ごとの制限の解釈調整 (仕様の対応表に準拠)。
            switch (type)
            {
                case JointType.Point2Point:
                    // 各制限/バネ無効。並進のみ固定。
                    j.AngularLowerLimit = new Vec3(1); // lo>hi = 回転フリー
                    j.AngularUpperLimit = new Vec3(-1);
                    j.LinearLowerLimit = j.LinearUpperLimit = Vec3.Zero;
                    j.SpringLinear = j.SpringAngular = Vec3.Zero;
                    break;

                case JointType.Hinge:
                    // X 軸のみ回転可。並進固定。
                    j.LinearLowerLimit = j.LinearUpperLimit = Vec3.Zero;
                    j.AngularLowerLimit = new Vec3(angLo.x, 0, 0);
                    j.AngularUpperLimit = new Vec3(angHi.x, 0, 0);
                    break;

                case JointType.Slider:
                    // X 軸のみ並進/回転可。
                    j.LinearLowerLimit = new Vec3(linLo.x, 0, 0);
                    j.LinearUpperLimit = new Vec3(linHi.x, 0, 0);
                    j.AngularLowerLimit = new Vec3(angLo.x, 0, 0);
                    j.AngularUpperLimit = new Vec3(angHi.x, 0, 0);
                    break;

                case JointType.ConeTwist:
                    // 並進固定。回転は円錐 (Y,Z) + 捻り (X)。
                    j.LinearLowerLimit = j.LinearUpperLimit = Vec3.Zero;
                    break;

                case JointType.Generic6Dof:
                    // バネ無効。
                    j.SpringLinear = j.SpringAngular = Vec3.Zero;
                    break;

                case JointType.Spring6Dof:
                default:
                    break;
            }
            return j;
        }

        private static bool IsLocked(float lo, float hi) => lo == hi;
        private static bool IsFree(float lo, float hi) => lo > hi;

        // --- 準備: フレーム/アンカー/行を構築 ---
        public void Prepare(float dt, bool splitBias = false, bool warmStart = false, bool warmStartAng = false)
        {
            // SplitCrossOnly が立っているときは、横渡し型だけ split・鎖型は既定 (実速度側) にする。
            _splitBias = SplitCrossOnly ? IsCrossTypeJoint
                       : SplitChainOnly ? !IsCrossTypeJoint
                       : splitBias;
            _warmStart = warmStart;
            _warmStartAng = warmStartAng;
            _warmLinSeen[0] = _warmLinSeen[1] = _warmLinSeen[2] = false;
            _warmAngSeen[0] = _warmAngSeen[1] = _warmAngSeen[2] = false;
            _rows.Clear();
            if (BodyA == null || BodyB == null) return;

            _worldA = BodyA.WorldTransform * FrameInA;
            _worldB = BodyB.WorldTransform * FrameInB;
            _anchorA = _worldA.Origin;
            _anchorB = _worldB.Origin;

            var rA = _anchorA - BodyA.CenterOfMass;
            var rB = _anchorB - BodyB.CenterOfMass;

            var basisA = Matrix3x3.FromQuat(_worldA.Rotation);
            _axesA[0] = basisA.Column(0);
            _axesA[1] = basisA.Column(1);
            _axesA[2] = basisA.Column(2);
            // 診断: 横渡し型の並進ロック軸をバインド時方向へ凍結する (既定OFF)。
            if (_bindAxes == null) { _bindAxes = new[] { _axesA[0], _axesA[1], _axesA[2] }; }
            bool freezeLin = FreezeCrossAxes && IsCrossTypeJoint;

            var invDt = dt > 0 ? 1f / dt : 0f;
            var linDelta = _anchorB - _anchorA;

            // 回転相対 (角度行と、LinearLeverMode=2 の rotAllowed 判定で使用)。
            var qRel = _worldA.Rotation.Conjugated() * _worldB.Rotation;
            // BulletAngleConvention: Bullet 2.75 は実挙動として R_B⁻¹R_A のオイラーを見ている
            // (btGetMatrixElem が列優先添字で、matrixToEulerXYZ が転置に対して働くため)。
            // 既定 false のときは従来どおり qRel をそのまま使う = ビット不変。
            var euler = ToEulerXYZ(BulletAngleConvention ? qRel.Conjugated().Normalized : qRel.Normalized);

            // LinearLeverMode=2 用の前計算 (Bullet calculateTransforms / setLinearLimits 相当)。
            bool hasStatic = false; float factA = 0.5f, factB = 0.5f;
            Span<bool> angActive = stackalloc bool[3];
            if (LinearLeverMode == 2)
            {
                float miA = BodyA.InverseMass, miB = BodyB.InverseMass;
                hasStatic = miA < 1e-9f || miB < 1e-9f;
                float miS = miA + miB;
                factA = miS > 0f ? miB / miS : 0.5f; factB = 1f - factA;
                for (int i = 0; i < 3; i++)
                {
                    float alo = AngularLowerLimit[i], ahi = AngularUpperLimit[i];
                    // Bullet testLimitValue 相当: free=0 / 範囲外 or ロック(誤差あり)=active
                    angActive[i] = !IsFree(alo, ahi) && (IsLocked(alo, ahi) ? euler[i] != alo : (euler[i] < alo || euler[i] > ahi));
                }
            }

            // --- 並進 3 軸 ---
            for (int i = 0; i < 3; i++)
            {
                var axis = freezeLin ? _bindAxes[i] : _axesA[i];
                float lo = LinearLowerLimit[i], hi = LinearUpperLimit[i];
                float curF = linDelta.Dot(axis);
                if (IsFree(lo, hi))
                {
                    // 自由軸 (lo>hi)。Bullet は limit=0 なのでモーターだけが行を作る。
                    if (SpringAsMotorRow) AddSpringMotorRow(false, i, axis, curF, lo, hi, rA, rB, invDt);
                    continue;
                }

                float cur = curF;
                float err, lower, upper;
                if (IsLocked(lo, hi)) { err = lo - cur; lower = -1e18f; upper = 1e18f; }
                else if (cur < lo) { err = lo - cur; lower = 0f; upper = 1e18f; }
                else if (cur > hi) { err = hi - cur; lower = -1e18f; upper = 0f; }
                else
                {
                    // 制限内 → Bullet は limit=0。モーター行だけが立つ (既定では後段の陽的ばね)。
                    if (SpringAsMotorRow) AddSpringMotorRow(false, i, axis, cur, lo, hi, rA, rB, invDt);
                    continue;
                }
                // デッドバンド: 誤差がフロア未満の行は作らない (既定 0 で無効=ビット不変)。
                if (ErrDeadband > 0f && Math.Abs(err) < ErrDeadband) continue;

                // レバーアームをモード別に決定 (LinearLeverMode コメント参照)。
                Vec3 armA = rA, armB = rB;
                if (LinearLeverMode == 1)
                {
                    armA = _anchorB - BodyA.CenterOfMass;   // 両剛体とも B側アンカー基準
                    armB = rB;
                }
                else if (LinearLeverMode == 2)
                {
                    // Bullet get_limit_motor_info2 (m_useOffsetForConstraintFrame) の翻訳分岐そのまま。
                    float pA = rA.Dot(axis), pB = rB.Dot(axis);
                    var orthoA = rA - axis * pA;
                    var orthoB = rB - axis * pB;
                    float desiredOffs = cur + err;               // = 目標バウンド (locked なら lo)
                    var totalDist = axis * (pA + desiredOffs - pB);
                    armA = orthoA + totalDist * factA;
                    armB = orthoB - totalDist * factB;
                    bool rotAllowed = !(angActive[(i + 1) % 3] && angActive[(i + 2) % 3]);
                    if (hasStatic && !rotAllowed) { armA *= factA; armB *= factB; }
                }

                // バイアスのみデッドバンド: 行は作るが位置補正成分だけ落とす (既定 0 で無効=ビット不変)。
                float linTarget = Clamp(err * Beta * invDt);
                if (BiasDeadband > 0f && Math.Abs(err) < BiasDeadband) linTarget = 0f;
                if (DebugRows != null)
                    DebugRows.Add((Name, i, false, err, linTarget,
                        ((BodyB.LinearVelocity + Vec3.Cross(BodyB.AngularVelocity, armB))
                       - (BodyA.LinearVelocity + Vec3.Cross(BodyA.AngularVelocity, armA))).Dot(axis)));
                AddLinearRow(axis, armA, armB, linTarget, lower, upper, i, IsLocked(lo, hi));
            }

            // --- 回転 3 軸 ---
            // Bullet混合軸 (AngularMixedAxes=true): calculateAngleInfo をそのまま再現。
            Vec3 mix0 = default, mix1 = default, mix2 = default;
            if (AngularMixedAxes)
            {
                var basisB = Matrix3x3.FromQuat(_worldB.Rotation);
                var axis0 = basisB.Column(0);   // B col0
                var axis2 = _axesA[2];          // A col2
                var m1 = Vec3.Cross(axis2, axis0);
                var m0 = Vec3.Cross(m1, axis2);
                var m2 = Vec3.Cross(axis0, m1);
                mix0 = m0.Normalized; mix1 = m1.Normalized; mix2 = m2.Normalized;
            }
            for (int i = 0; i < 3; i++)
            {
                var axis = AngularMixedAxes ? (i == 0 ? mix0 : i == 1 ? mix1 : mix2) : _axesA[i];
                // BulletAngleConvention の (2): cur が R_B⁻¹R_A 側になると d(cur)/dt = (wA-wB)·axis に
                // なるので、相対速度を (wB-wA)·axis で測るこの行と符号を合わせるため軸を反転する。
                // これで err の作り方・不等式の側・力積の上下限が Bullet と1対1に対応する。
                if (BulletAngleConvention) axis = -axis;
                float lo = AngularLowerLimit[i], hi = AngularUpperLimit[i];
                if (IsFree(lo, hi))
                {
                    DebugAngularRows?.Add((Name, i, 0, 0f, 0f));
                    if (SpringAsMotorRow) AddSpringMotorRow(true, i, axis, euler[i], lo, hi, Vec3.Zero, Vec3.Zero, invDt);
                    continue;
                }

                float cur = euler[i];
                float err, lower, upper;
                int sideCode;
                if (IsLocked(lo, hi)) { err = lo - cur; lower = -1e18f; upper = 1e18f; sideCode = 0; }
                else if (cur < lo) { err = lo - cur; lower = 0f; upper = 1e18f; sideCode = 1; }
                else if (cur > hi) { err = hi - cur; lower = -1e18f; upper = 0f; sideCode = 2; }
                else
                {
                    DebugAngularRows?.Add((Name, i, 1, cur, 0f));
                    if (SpringAsMotorRow) AddSpringMotorRow(true, i, axis, cur, lo, hi, Vec3.Zero, Vec3.Zero, invDt);
                    continue;
                }
                if (ErrDeadband > 0f && Math.Abs(err) < ErrDeadband) continue;

                DebugAngularRows?.Add((Name, i, IsLocked(lo, hi) ? 2 : (cur < lo ? 3 : 4), cur, err));
                float angBeta = Beta * AngularBetaScale;   // 既定 1 倍 = Beta と厳密に同値
                if (DebugRows != null)
                    DebugRows.Add((Name, i, true, err, Clamp(err * angBeta * invDt),
                        (BodyB.AngularVelocity - BodyA.AngularVelocity).Dot(axis)));
                AddAngularRow(axis, Clamp(err * angBeta * invDt), lower, upper, i, sideCode);
            }

            // warm-start: 今サブステップで warm 対象にならなかった DOF の蓄積は破棄する
            // (常時アクティブでない DOF の古い力積を誤って引き継がないため)。
            if (_warmStart) for (int i = 0; i < 3; i++) if (!_warmLinSeen[i]) _warmLin[i] = 0f;
            if (_warmStartAng) for (int i = 0; i < 3; i++) if (!_warmAngSeen[i]) { _warmAng[i] = 0f; _warmAngPrevSide[i] = -1; }

            // 前サブステップの蓄積インパルスを反復前に一度適用する (Bullet の warm-start と同型)。
            if (_warmStart || _warmStartAng)
            {
                for (int r = 0; r < _rows.Count; r++)
                {
                    var row = _rows[r];
                    if (!row.WarmStartable) continue;
                    if (row.Angular)
                    {
                        var L = row.Axis * row.Accumulated;
                        BodyA.ApplyTorqueImpulse(-L);
                        BodyB.ApplyTorqueImpulse(L);
                    }
                    else
                    {
                        var P = row.Axis * row.Accumulated;
                        BodyA.ApplyImpulse(-P, row.RelA);
                        BodyB.ApplyImpulse(P, row.RelB);
                    }
                }
            }
        }


        /// <summary>ばねを Bullet 2.75 のリミットモーター行として1本足す (SpringAsMotorRow=ON のとき)。
        /// 呼ぶのは **リミット行が立たなかった軸だけ** (Bullet の `if (powered) { if (!limit) {...} }` と同じ)。
        /// ロック軸では Bullet が powered=0 にするので呼ばない。</summary>
        private void AddSpringMotorRow(bool angular, int i, Vec3 axis, float cur, float lo, float hi,
                                       Vec3 rA, Vec3 rB, float invDt)
        {
            float k = angular ? SpringAngular[i] : SpringLinear[i];
            if (k <= 0f) return;
            float fps = invDt;                                   // = 1/サブステップdt
            if (fps <= 0f) return;
            float eq = ClampToLimit(0f, lo, hi);                 // 平衡点 (当エンジンの従来定義)
            float delta = cur - eq;
            // velFactor = fps * springDamping / numIterations   (Bullet 実ソース)
            int iters = SolverIterationsForSpring > 0 ? SolverIterationsForSpring : 10;
            float velFactor = fps * BulletSpringDamping / iters;
            // 目標速度を **当エンジンの符号規約** で直に作る:
            //   cur の時間微分 = この行の相対速度なので、delta を減らす向き = -velFactor*delta*k
            //   (Bullet の linear force=delta*k / angular force=-delta*k を各々の符号規約へ通したものと同値)
            float force = delta * k;
            float targetVel = -velFactor * force;
            float maxMotorForce = Math.Abs(force) / fps;
            // リミット手前で先細り (Bullet getMotorFactor)。timeFact = fps * erp、erp は Beta。
            float motFact = MotorFactor(cur, lo, hi, targetVel, fps * Beta);
            targetVel *= motFact;
            // ★モーター行にもリミット行と同じレバーアーム規約を適用する。
            //   Bullet の get_limit_motor_info2 は `if(!rotational)` の Linear Torque Decoupling を
            //   モーター行にも通すので、LinearLeverMode=1 のときは同じ扱いにしないと実効質量がずれる。
            //   既定 (LeverMode=0) では armA=rA / armB=rB なのでビット不変。
            //   ★診断フックより **前** に決めること: ここを後回しにすると DebugRows の相対速度だけが
            //     補正前の rA で出て、アンカーの離れた横渡し型で最大90%の偽の不一致になる (タスク34)。
            Vec3 armA = rA, armB = rB;
            if (LinearLeverMode == 1) armA = _anchorB - BodyA.CenterOfMass;
            // 診断フック: モーター行も限界行と同じスキーマで出す (err の欄には delta を入れる)。
            //   これが無いと ROWTRACE の行数がソルバの行数と合わない (タスク34)。
            if (DebugRows != null)
                DebugRows.Add((Name, i, angular, delta, targetVel,
                    angular ? (BodyB.AngularVelocity - BodyA.AngularVelocity).Dot(axis)
                            : ((BodyB.LinearVelocity + Vec3.Cross(BodyB.AngularVelocity, armB))
                             - (BodyA.LinearVelocity + Vec3.Cross(BodyA.AngularVelocity, armA))).Dot(axis)));
            if (angular) { AddAngularRow(axis, targetVel, -maxMotorForce, maxMotorForce, i, 5); return; }
            AddLinearRow(axis, armA, armB, targetVel, -maxMotorForce, maxMotorForce, i, false);
        }

        /// <summary>モーター行の velFactor に要る反復回数。PhysicsWorld が毎サブステップ写す。
        /// 既定 0 のときは 10 (Bullet 既定) を使う。</summary>
        public static int SolverIterationsForSpring = 0;

        private static float Clamp(float v) =>
            Math.Max(-MaxCorrectionVel, Math.Min(MaxCorrectionVel, v));

        private void AddLinearRow(Vec3 axis, Vec3 rA, Vec3 rB, float targetVel, float lo, float hi, int dof, bool locked)
        {
            var rAxn = Vec3.Cross(rA, axis);
            var rBxn = Vec3.Cross(rB, axis);
            float k = BodyA.InverseMass + BodyB.InverseMass
                    + rAxn.Dot(BodyA.InverseInertiaWorld * rAxn)
                    + rBxn.Dot(BodyB.InverseInertiaWorld * rBxn);
            // (a-1) 直線ロック行のみ warm-start: 前サブステップの蓄積で初期化する。
            bool warm = _warmStart && locked;
            float acc = 0f;
            if (warm) { _warmLinSeen[dof] = true; acc = Math.Max(lo, Math.Min(hi, _warmLin[dof] * WarmStartFactor)); }
            _rows.Add(new ConstraintRow
            {
                Axis = axis, Angular = false, RelA = rA, RelB = rB,
                LowerImpulse = lo, UpperImpulse = hi,
                TargetVel = _splitBias ? 0f : targetVel,
                PositionBias = _splitBias ? targetVel : 0f,
                EffMass = k > 0 ? 1f / k : 0f,
                Dof = dof, WarmStartable = warm, Accumulated = acc,
            });
        }

        private void AddAngularRow(Vec3 axis, float targetVel, float lo, float hi, int dof, int sideCode)
        {
            float k = axis.Dot(BodyA.InverseInertiaWorld * axis)
                    + axis.Dot(BodyB.InverseInertiaWorld * axis);
            // (a-2) 角度 warm-start: 前サブステップと同じ側(sideCode)なら蓄積を引き継ぎ、
            // 側が変わった/不在だった場合は破棄(0スタート)。トグル頻度を診断カウント。
            bool warm = false;
            float acc = 0f;
            if (_warmStartAng)
            {
                warm = true;
                _warmAngSeen[dof] = true;
                WarmAngRows++;
                bool sameSide = _warmAngPrevSide[dof] == sideCode;
                if (!sameSide) WarmAngToggles++;
                acc = sameSide ? Math.Max(lo, Math.Min(hi, _warmAng[dof] * WarmStartFactor)) : 0f;
                _warmAngPrevSide[dof] = sideCode;
            }
            _rows.Add(new ConstraintRow
            {
                Axis = axis, Angular = true,
                LowerImpulse = lo, UpperImpulse = hi,
                TargetVel = _splitBias ? 0f : targetVel,
                PositionBias = _splitBias ? targetVel : 0f,
                EffMass = k > 0 ? 1f / k : 0f,
                Dof = dof, WarmStartable = warm, Accumulated = acc,
            });
        }

        // --- バネ (明示力積, サブステップ毎に 1 回) ---
        //
        // ★安定化クランプ (2026-08-10)。
        //   陽的(前進オイラー)ばね impulse = -k*err*dt は k*dt²/m が 1 を超えると必ず発散する。
        //   1ステップの速度変化が k*err*dt/m、それが次ステップの誤差を |err| より大きくするため、
        //   誤差が毎ステップ k*dt²/m 倍に増幅されるからである。
        //   実測: ぬこ式レーシングミク2023 は spring=100000 に対し錘の質量 0.01 で
        //   k*dt²/m = 2778 → 毎ステップ約30倍に発散し、25ステップで float が溢れて NaN になった
        //   (ツインテール先端の錘が起点。重力ゼロでも発散するので外力ではなくばねが原因)。
        //   PMXモデルの 6/13 が同じ発散域にあり、これまで表面化しなかったのは忠実度検証に使っていた
        //   IA のジョイントがばね定数ゼロだったため（＝1モデルでの検証は安定性を保証しない）。
        //
        //   対策: 力積を「1ステップで誤差をちょうど打ち消す量」= |err| * mEff / dt で頭打ちにする。
        //   これは k*dt²/m = 1（デッドビート）に相当する安定限界で、行き過ぎが原理的に起きない。
        //   mEff はソルバ本体 (AddLinearRow / AddAngularRow) と同一の式で求めるため、
        //   クランプが効かない範囲 (k*dt²/m < 1) では従来と1ビットも変わらない。
        //   実測A/B(300step 全剛体姿勢ハッシュ): IA系3モデルはビット完全一致、Racing_Miku2023 は NaN→完走。
        public void ApplySprings(float dt)
        {
            if (BodyA == null || BodyB == null) return;
            bool hasLin = SpringLinear.LengthSquared > 0;
            bool hasAng = SpringAngular.LengthSquared > 0;
            if (!hasLin && !hasAng) return;
            float invDtSpring = dt > 0 ? 1f / dt : 0f;

            var rA = _anchorA - BodyA.CenterOfMass;
            var rB = _anchorB - BodyB.CenterOfMass;
            var linDelta = _anchorB - _anchorA;

            // ★既定 (SpringAsMotorRow = true) はここで抜ける。以降の陽的力積と
            //   ClampSpringImpulse は **フラグOFF時の対照経路** であって出荷経路ではない。
            if (SpringAsMotorRow) return;   // モーター行で解くので陽的経路は通さない (二重適用の防止)

            if (hasLin)
            {
                for (int i = 0; i < 3; i++)
                {
                    float k = SpringLinear[i];
                    if (k <= 0) continue;
                    var axis = _axesA[i];
                    float eq = ClampToLimit(0f, LinearLowerLimit[i], LinearUpperLimit[i]);
                    float err = linDelta.Dot(axis) - eq;
                    // Bullet の 6DOF バネには速度比例の粘性項が無いので付けない (force = -delta*k のみ)。
                    float impulse = (-k * err) * dt;
                    // 安定化: AddLinearRow と同一の実効質量でデッドビート量に頭打ち。
                    var rAxn = Vec3.Cross(rA, axis);
                    var rBxn = Vec3.Cross(rB, axis);
                    float invMEff = BodyA.InverseMass + BodyB.InverseMass
                                  + rAxn.Dot(BodyA.InverseInertiaWorld * rAxn)
                                  + rBxn.Dot(BodyB.InverseInertiaWorld * rBxn);
                    impulse = ClampSpringImpulse(impulse, err, invMEff, invDtSpring);
                    var P = axis * impulse;
                    BodyA.ApplyImpulse(-P, rA);
                    BodyB.ApplyImpulse(P, rB);
                }
            }

            if (hasAng)
            {
                var qRel = _worldA.Rotation.Conjugated() * _worldB.Rotation;
                var euler = ToEulerXYZ(qRel.Normalized);
                for (int i = 0; i < 3; i++)
                {
                    float k = SpringAngular[i];
                    if (k <= 0) continue;
                    var axis = _axesA[i];
                    float eq = ClampToLimit(0f, AngularLowerLimit[i], AngularUpperLimit[i]);
                    float err = euler[i] - eq;
                    // Bullet の 6DOF バネには速度比例の粘性項が無いので付けない (force = -delta*k のみ)。
                    float impulse = (-k * err) * dt;
                    // 安定化: AddAngularRow と同一の実効慣性でデッドビート量に頭打ち。
                    float invMEff = axis.Dot(BodyA.InverseInertiaWorld * axis)
                                  + axis.Dot(BodyB.InverseInertiaWorld * axis);
                    impulse = ClampSpringImpulse(impulse, err, invMEff, invDtSpring);
                    var L = axis * impulse;
                    BodyA.ApplyTorqueImpulse(-L);
                    BodyB.ApplyTorqueImpulse(L);
                }
            }
        }

        /// <summary>陽的ばねの力積を安定限界（デッドビート＝1ステップで誤差をちょうど打ち消す量）で
        /// 頭打ちにする。invMEff はソルバ行と同じ「1/実効質量」。これを超える力積は必ず行き過ぎを生み、
        /// 誤差が毎ステップ増幅されて発散する。k*dt²/m &lt; 1 の健全なモデルでは一度も発動しない。
        ///
        /// ★★これは **陽的経路 (<see cref="SpringAsMotorRow"/> = false) 専用の絆創膏** である。
        ///   既定は 2026-08-22 (タスク37) から SpringAsMotorRow = true なので、
        ///   **既定経路ではこの関数は一度も呼ばれない** (<see cref="ApplySprings"/> が冒頭で return する)。
        ///   モーター行は力積の上下限が ±maxMotorForce で構造的に閉じているため、この頭打ちを必要としない。
        ///   実測: モデルR は陽的経路だとクランプ発動 30.5% かつ無効化で NaN になるが、
        ///   モーター行では発動0件・NaN なしで完走する。
        ///   陽的経路は A/B と回帰の対照として残置しており、削除しない。</summary>
        private static float ClampSpringImpulse(float impulse, float err, float invMEff, float invDt)
        {
            if (invMEff <= 0f || invDt <= 0f) return impulse; // 両方静的 等
            float maxImp = Math.Abs(err) / invMEff * invDt;   // |err| * mEff / dt
            float clamped = impulse > maxImp ? maxImp : (impulse < -maxImp ? -maxImp : impulse);
            if (CollectSpringClampStats)
            {
                SpringRows++;
                SpringImpulseAbsSum += Math.Abs(impulse);
                if (clamped != impulse) { SpringClamped++; SpringShavedSum += Math.Abs(impulse - clamped); }
            }
            return DisableSpringClamp ? impulse : clamped;
        }

        /// <summary>診断専用 (既定 false = ビット不変)。陽的ばねのデッドビート・クランプを無効化する。
        /// タスク27: モデルPのネクタイが参照(PMXエディタ)では揺れ続けるのに当エンジンでは凍結する
        /// (フロアの1/40) 件で、「クランプが犯人か」を白黒つけるためのスイッチ。
        /// ★クランプの当初の存在理由は k*dt²/m が大きいモデルでの発散 (モデルR の NaN) なので、
        ///   無効化して測るときは **必ず NaN 監視を同時に回すこと**。出荷候補ではない。</summary>
        public static bool DisableSpringClamp = false;

        /// <summary>診断専用 (既定 false = ビット不変)。ばねクランプの発動頻度と削り量を数える。
        /// 挙動には影響しない (カウンタのみ)。読み出し前に <see cref="ResetSpringClampStats"/> を呼ぶこと。</summary>
        public static bool CollectSpringClampStats = false;
        public static long SpringRows, SpringClamped;
        public static double SpringImpulseAbsSum, SpringShavedSum;
        public static void ResetSpringClampStats()
        { SpringRows = 0; SpringClamped = 0; SpringImpulseAbsSum = 0; SpringShavedSum = 0; }

        private static float ClampToLimit(float v, float lo, float hi)
        {
            if (lo > hi) return v;      // free
            return Math.Max(lo, Math.Min(hi, v));
        }

        // --- 速度反復 (world から複数回呼ばれる) ---
        public void SolveVelocity()
        {
            for (int r = 0; r < _rows.Count; r++)
            {
                var row = _rows[r];
                // 線形行は行のレバーアーム(RelA/RelB)で速度を測る (J整合)。mode0 では
                // RelA/RelB = anchor−COM なので従来の VelocityAtPoint(anchor) とビット同一。
                float relVel = row.Angular
                    ? (BodyB.AngularVelocity - BodyA.AngularVelocity).Dot(row.Axis)
                    : ((BodyB.LinearVelocity + Vec3.Cross(BodyB.AngularVelocity, row.RelB))
                     - (BodyA.LinearVelocity + Vec3.Cross(BodyA.AngularVelocity, row.RelA))).Dot(row.Axis);

                float dImpulse = (row.TargetVel - relVel) * row.EffMass;
                float old = row.Accumulated;
                row.Accumulated = Math.Max(row.LowerImpulse, Math.Min(row.UpperImpulse, old + dImpulse));
                dImpulse = row.Accumulated - old;

                if (row.Angular)
                {
                    var L = row.Axis * dImpulse;
                    BodyA.ApplyTorqueImpulse(-L);
                    BodyB.ApplyTorqueImpulse(L);
                }
                else
                {
                    var P = row.Axis * dImpulse;
                    BodyA.ApplyImpulse(-P, row.RelA);
                    BodyB.ApplyImpulse(P, row.RelB);
                }
                // warm-start: 行の最終累積を次サブステップへ引き継ぐ (角度→_warmAng / 直線→_warmLin)。
                if (row.WarmStartable) { if (row.Angular) _warmAng[row.Dof] = row.Accumulated; else _warmLin[row.Dof] = row.Accumulated; }
                if (DebugRowsSolved != null && (DebugRowsSolvedJoint == null || Name.Contains(DebugRowsSolvedJoint)))
                {
                    float after = row.Angular
                        ? (BodyB.AngularVelocity - BodyA.AngularVelocity).Dot(row.Axis)
                        : ((BodyB.LinearVelocity + Vec3.Cross(BodyB.AngularVelocity, row.RelB))
                         - (BodyA.LinearVelocity + Vec3.Cross(BodyA.AngularVelocity, row.RelA))).Dot(row.Axis);
                    DebugRowsSolved.Add((Name, row.Dof, row.Angular, row.Axis, row.RelA, row.RelB,
                        BodyA.Name, BodyB.Name, row.Accumulated, row.TargetVel, after));
                }
                _rows[r] = row;
            }
        }

        // --- 位置補正の split-impulse 反復 (擬似速度側)。UseJointSplitImpulse 有効時のみ world から呼ばれる ---
        // 実速度は SolveVelocity が target=0 (バイアス抜き) で解き、位置誤差 err の補正はここで
        // 擬似速度へ積む。擬似速度は位置積分にのみ反映され実速度には残らないため、Baumgarte が
        // 実速度へエネルギーを注入せず、以後の warm-start を安定化できる (Bullet の split-impulse と同型)。
        public void SolveSplitPosition()
        {
            for (int r = 0; r < _rows.Count; r++)
            {
                var row = _rows[r];
                float relVel = row.Angular
                    ? (BodyB.PseudoAngularVelocity - BodyA.PseudoAngularVelocity).Dot(row.Axis)
                    : ((BodyB.PseudoLinearVelocity + Vec3.Cross(BodyB.PseudoAngularVelocity, row.RelB))
                     - (BodyA.PseudoLinearVelocity + Vec3.Cross(BodyA.PseudoAngularVelocity, row.RelA))).Dot(row.Axis);

                float dImpulse = (row.PositionBias - relVel) * row.EffMass;
                float old = row.PseudoAccumulated;
                row.PseudoAccumulated = Math.Max(row.LowerImpulse, Math.Min(row.UpperImpulse, old + dImpulse));
                dImpulse = row.PseudoAccumulated - old;

                if (row.Angular)
                {
                    var L = row.Axis * dImpulse;
                    BodyA.ApplyPushTorqueImpulse(-L);
                    BodyB.ApplyPushTorqueImpulse(L);
                }
                else
                {
                    var P = row.Axis * dImpulse;
                    BodyA.ApplyPushImpulse(-P, row.RelA);
                    BodyB.ApplyPushImpulse(P, row.RelB);
                }
                _rows[r] = row;
            }
        }

        // --- Euler XYZ 抽出 (Bullet の matrixToEulerXYZ 相当) ---
        internal static Vec3 ToEulerXYZ(Quat q)
        {
            var m = Matrix3x3.FromQuat(q);
            // 行列から XYZ オイラー角を復元。
            float m02 = m.Row0.z;
            float y, x, z;
            if (m02 < 1f - 1e-6f)
            {
                if (m02 > -1f + 1e-6f)
                {
                    y = (float)Math.Asin(Math.Clamp(m02, -1f, 1f));
                    x = (float)Math.Atan2(-m.Row1.z, m.Row2.z);
                    z = (float)Math.Atan2(-m.Row0.y, m.Row0.x);
                }
                else
                {
                    y = -(float)(Math.PI / 2);
                    x = -(float)Math.Atan2(m.Row1.x, m.Row1.y);
                    z = 0f;
                }
            }
            else
            {
                y = (float)(Math.PI / 2);
                x = (float)Math.Atan2(m.Row1.x, m.Row1.y);
                z = 0f;
            }
            return new Vec3(x, y, z);
        }
    }
}
