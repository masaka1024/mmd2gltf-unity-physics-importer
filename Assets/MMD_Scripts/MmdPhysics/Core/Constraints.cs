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
        // warm-start 引継ぎ係数 (Bullet m_warmstartingFactor 相当, 既定0.85)。過拘束系で
        // 引継ぎインパルスがラチェット的に積み上がるのを抑える。1.0=減衰なし(積み上がりやすい)。
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
            _splitBias = splitBias;
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

            var invDt = dt > 0 ? 1f / dt : 0f;
            var linDelta = _anchorB - _anchorA;

            // 回転相対 (角度行と、LinearLeverMode=2 の rotAllowed 判定で使用)。
            var qRel = _worldA.Rotation.Conjugated() * _worldB.Rotation;
            var euler = ToEulerXYZ(qRel.Normalized);

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
                var axis = _axesA[i];
                float lo = LinearLowerLimit[i], hi = LinearUpperLimit[i];
                if (IsFree(lo, hi)) continue;

                float cur = linDelta.Dot(axis);
                float err, lower, upper;
                if (IsLocked(lo, hi)) { err = lo - cur; lower = -1e18f; upper = 1e18f; }
                else if (cur < lo) { err = lo - cur; lower = 0f; upper = 1e18f; }
                else if (cur > hi) { err = hi - cur; lower = -1e18f; upper = 0f; }
                else continue; // 制限内 → バネのみ (後段)

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

                AddLinearRow(axis, armA, armB, Clamp(err * Beta * invDt), lower, upper, i, IsLocked(lo, hi));
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
                float lo = AngularLowerLimit[i], hi = AngularUpperLimit[i];
                if (IsFree(lo, hi)) continue;

                float cur = euler[i];
                float err, lower, upper;
                int sideCode;
                if (IsLocked(lo, hi)) { err = lo - cur; lower = -1e18f; upper = 1e18f; sideCode = 0; }
                else if (cur < lo) { err = lo - cur; lower = 0f; upper = 1e18f; sideCode = 1; }
                else if (cur > hi) { err = hi - cur; lower = -1e18f; upper = 0f; sideCode = 2; }
                else continue;

                AddAngularRow(axis, Clamp(err * Beta * invDt), lower, upper, i, sideCode);
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
        /// 誤差が毎ステップ増幅されて発散する。k*dt²/m &lt; 1 の健全なモデルでは一度も発動しない。</summary>
        private static float ClampSpringImpulse(float impulse, float err, float invMEff, float invDt)
        {
            if (invMEff <= 0f || invDt <= 0f) return impulse; // 両方静的 等
            float maxImp = Math.Abs(err) / invMEff * invDt;   // |err| * mEff / dt
            if (impulse > maxImp) return maxImp;
            if (impulse < -maxImp) return -maxImp;
            return impulse;
        }

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
