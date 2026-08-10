// ===========================================================================
// Unity Bullet 互換物理エンジン – RigidBody
// Bullet の btRigidBody に対応。PMX 剛体パラメータを保持。
// ===========================================================================

using System;

namespace BulletPhysics
{
    /// <summary>
    /// PMX 剛体の物理演算タイプ。
    /// 0:ボーン追従(static/kinematic) 1:物理演算(dynamic) 2:物理演算+Bone位置合わせ
    /// </summary>
    public enum PhysicsMode
    {
        BoneFollow = 0,   // Kinematic: ボーンから位置を与える
        Dynamic = 1,      // 物理演算で動く
        DynamicBoneMerge = 2, // 物理演算後に位置をボーンへ合わせる
    }

    /// <summary>
    /// 剛体。btRigidBody 相当。運動状態 (姿勢/速度) と物性を保持する。
    /// </summary>
    public sealed class RigidBody
    {
        // --- 識別/PMX メタ ---
        public string Name = string.Empty;
        public int Index = -1;
        public int BoneIndex = -1;

        // グループ / 衝突マスク (PMX)。マスクは bit=1 で「そのグループと衝突する」。
        public byte Group;
        public ushort CollisionMask;

        // --- 形状/物性 ---
        public CollisionShape Shape;
        public PhysicsMode Mode = PhysicsMode.Dynamic;

        public float Mass;               // 質量 (static/kinematic は 0 扱い)
        public float LinearDamping;      // 移動減衰
        public float AngularDamping;     // 回転減衰
        public float Restitution;        // 反発力
        public float Friction;           // 摩擦力

        // --- 運動状態 ---
        public RigidTransform WorldTransform = RigidTransform.Identity;
        public Vec3 LinearVelocity;
        public Vec3 AngularVelocity;

        // Split Impulse 用の擬似速度 (btRigidBody の m_pushVelocity / m_turnVelocity 相当)。
        // 貫入回復専用で実速度とは独立。毎サブステップ 0 に初期化され、位置積分に足すが速度としては残さない。
        public Vec3 PseudoLinearVelocity;
        public Vec3 PseudoAngularVelocity;

        // ボーン追従(kinematic) の目標姿勢 (Unity 側から毎フレーム設定)。
        public RigidTransform KinematicTarget = RigidTransform.Identity;

        // サブステップ毎に補間された当該サブステップの目標姿勢 (PhysicsWorld が設定)。
        public RigidTransform KinematicStepTarget = RigidTransform.Identity;

        // 剛体重心オフセット (PMX 剛体はボーン基準に配置される。ここでは原点=重心とする)。
        public Vec3 LocalInertiaDiag;    // ローカル慣性テンソル対角

        // 逆質量・逆慣性 (ワールド)。
        public float InverseMass;
        public Matrix3x3 InverseInertiaWorld = Matrix3x3.Zero;
        private Vec3 _inverseInertiaLocal;

        // 累積力 (積分時に適用)。
        public Vec3 TotalForce;
        public Vec3 TotalTorque;

        // インパルスモーフ用の保持値 (ローカル/グローバル別)。
        public Vec3 ImpulseLinearLocal, ImpulseAngularLocal;
        public Vec3 ImpulseLinearGlobal, ImpulseAngularGlobal;
        public bool ImpulseResetFlag;

        // Sleep 管理。
        public bool IsActive = true;
        public float SleepTimer;

        public bool IsStaticOrKinematic =>
            Mode == PhysicsMode.BoneFollow || Mass <= 0f;

        public bool IsKinematic => Mode == PhysicsMode.BoneFollow;

        public RigidBody(CollisionShape shape)
        {
            Shape = shape;
            SetMassProps(0f);
        }

        /// <summary>質量から逆質量・ローカル逆慣性を設定する。</summary>
        public void SetMassProps(float mass)
        {
            Mass = mass;
            if (mass > 0f && !IsKinematic)
            {
                InverseMass = 1f / mass;
                LocalInertiaDiag = Shape.CalculateLocalInertia(mass);
                _inverseInertiaLocal = new Vec3(
                    LocalInertiaDiag.x > 0 ? 1f / LocalInertiaDiag.x : 0f,
                    LocalInertiaDiag.y > 0 ? 1f / LocalInertiaDiag.y : 0f,
                    LocalInertiaDiag.z > 0 ? 1f / LocalInertiaDiag.z : 0f);
            }
            else
            {
                InverseMass = 0f;
                LocalInertiaDiag = Vec3.Zero;
                _inverseInertiaLocal = Vec3.Zero;
            }
            UpdateInertiaWorld();
        }

        /// <summary>ワールド逆慣性テンソルを姿勢から更新する。</summary>
        public void UpdateInertiaWorld()
        {
            if (InverseMass == 0f)
            {
                InverseInertiaWorld = Matrix3x3.Zero;
                return;
            }
            var basis = Matrix3x3.FromQuat(WorldTransform.Rotation);
            // R * diag(invI) * R^T
            InverseInertiaWorld = basis.Scaled(_inverseInertiaLocal);
        }

        // --- ヘルパー ---

        public Vec3 CenterOfMass => WorldTransform.Origin;

        /// <summary>剛体上の点 (ワールド) の速度。v + w × r。</summary>
        public Vec3 VelocityAtPoint(Vec3 worldPoint)
        {
            var r = worldPoint - CenterOfMass;
            return LinearVelocity + Vec3.Cross(AngularVelocity, r);
        }

        public void ApplyForce(Vec3 force) => TotalForce += force;

        public void ApplyTorque(Vec3 torque) => TotalTorque += torque;

        public void ApplyCentralImpulse(Vec3 impulse)
        {
            if (InverseMass == 0f) return;
            LinearVelocity += impulse * InverseMass;
        }

        public void ApplyTorqueImpulse(Vec3 torque)
        {
            if (InverseMass == 0f) return;
            AngularVelocity += InverseInertiaWorld * torque;
        }

        /// <summary>点 rel (重心相対) に加える力積。並進+回転の両方に反映。</summary>
        public void ApplyImpulse(Vec3 impulse, Vec3 rel)
        {
            if (InverseMass == 0f) return;
            LinearVelocity += impulse * InverseMass;
            AngularVelocity += InverseInertiaWorld * Vec3.Cross(rel, impulse);
        }

        /// <summary>Split Impulse: 擬似速度へ加える貫入回復力積 (実速度には反映しない)。</summary>
        public void ApplyPushImpulse(Vec3 impulse, Vec3 rel)
        {
            if (InverseMass == 0f) return;
            PseudoLinearVelocity += impulse * InverseMass;
            PseudoAngularVelocity += InverseInertiaWorld * Vec3.Cross(rel, impulse);
        }

        /// <summary>Split Impulse: 擬似角速度へ加える純トルク力積 (実速度には反映しない)。ジョイントの角度位置補正用。</summary>
        public void ApplyPushTorqueImpulse(Vec3 torque)
        {
            if (InverseMass == 0f) return;
            PseudoAngularVelocity += InverseInertiaWorld * torque;
        }

        /// <summary>剛体上の点 (ワールド) の擬似速度。Split Impulse の反復で使用。</summary>
        public Vec3 PseudoVelocityAtPoint(Vec3 worldPoint)
        {
            var r = worldPoint - CenterOfMass;
            return PseudoLinearVelocity + Vec3.Cross(PseudoAngularVelocity, r);
        }

        public void ClearForces()
        {
            TotalForce = Vec3.Zero;
            TotalTorque = Vec3.Zero;
        }

        /// <summary>ワールド AABB を計算する。</summary>
        public Aabb ComputeAabb()
        {
            var c = WorldTransform.Origin;
            var r = Shape.BoundingRadius + Shape.Margin;
            return new Aabb(c - new Vec3(r), c + new Vec3(r));
        }
    }
}
