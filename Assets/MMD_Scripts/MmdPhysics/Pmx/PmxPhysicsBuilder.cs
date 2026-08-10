// ===========================================================================
// Unity Bullet 互換物理エンジン – PMX -> PhysicsWorld ビルダー
// PMX の剛体/Joint を物理エンジンのインスタンスへ変換する。
// 剛体<->ボーンのオフセット (バインドポーズ) も生成する。
// ===========================================================================

using System.Collections.Generic;

namespace BulletPhysics.Pmx
{
    /// <summary>剛体とボーンの紐付け。ボーン追従 / 物理フィードバックに使う。</summary>
    public struct BoneLink
    {
        public RigidBody Body;
        public int BoneIndex;
        // バインド時の「ボーン→剛体」相対変換 (bone^-1 * body)。
        public RigidTransform BodyOffsetFromBone;
        public PhysicsMode Mode;
    }

    public sealed class PmxPhysicsBuilder
    {
        public PhysicsWorld World { get; } = new();
        public readonly List<BoneLink> BoneLinks = new();
        public readonly List<RigidBody> Bodies = new();
        private PmxPhysicsModel _model;   // FK-rest計算用にボーン階層を参照

        public static PmxPhysicsBuilder Build(PmxPhysicsModel model)
        {
            var b = new PmxPhysicsBuilder();
            b._model = model;
            b.BuildBodies(model);
            b.BuildJoints(model);
            return b;
        }

        private void BuildBodies(PmxPhysicsModel model)
        {
            foreach (var rb in model.RigidBodies)
            {
                var shape = CreateShape(rb);
                var body = new RigidBody(shape)
                {
                    Name = rb.Name,
                    BoneIndex = rb.BoneIndex,
                    Group = rb.Group,
                    // PMX の 16bit フィールドは bit=1 が「そのグループと衝突する」を意味するので
                    // そのまま衝突マスクとして渡す (Bullet の collision mask 相当)。
                    CollisionMask = rb.NonCollisionGroup,
                    Mode = (PhysicsMode)rb.PhysicsMode,
                    LinearDamping = rb.LinearDamping,
                    AngularDamping = rb.AngularDamping,
                    Restitution = rb.Restitution,
                    Friction = rb.Friction,
                    WorldTransform = RigidTransform.FromEuler(rb.Position, rb.Rotation),
                };
                body.KinematicTarget = body.WorldTransform;
                // ボーン追従は質量 0 (kinematic)、それ以外は PMX 質量。
                body.SetMassProps(body.Mode == PhysicsMode.BoneFollow ? 0f : rb.Mass);

                World.AddBody(body);
                Bodies.Add(body);

                // ボーンオフセット。
                var link = new BoneLink
                {
                    Body = body,
                    BoneIndex = rb.BoneIndex,
                    Mode = body.Mode,
                    BodyOffsetFromBone = ComputeOffset(model, rb),
                };
                BoneLinks.Add(link);
                if (link.Mode == PhysicsMode.DynamicBoneMerge) _hasBoneMerge = true;
            }
        }

        /// <summary>
        /// 物理開始/リセット時に、動的剛体も含む全剛体を現在のボーン姿勢へ整合させる
        /// (MMD の物理演算リセット相当)。剛体を boneWorld * BodyOffsetFromBone に置き、
        /// 速度を 0、慣性ワールドを更新、接触/蓄積インパルスをクリアする。
        /// これをしないと、フレーム0で脚が曲がっている場合に kinematic な脚コライダーだけが
        /// フレーム0へ動き、動的スカートがバインド位置に取り残されて逃げられない貫入平衡に落ちる。
        ///
        /// getBoneWorld: ボーンindex → そのボーンのワールド姿勢 (無ければ null)。
        ///   null を返したボーン (BoneIndex&lt;0 や、姿勢が得られないボーン) はバインド位置のままとする。
        /// </summary>
        public void ResetBodiesToBonePose(System.Func<int, RigidTransform?> getBoneWorld)
        {
            foreach (var link in BoneLinks)
            {
                var body = link.Body;
                if (link.BoneIndex >= 0)
                {
                    var bw = getBoneWorld(link.BoneIndex);
                    if (bw.HasValue)
                    {
                        body.WorldTransform = bw.Value * link.BodyOffsetFromBone;
                        body.KinematicTarget = body.WorldTransform;
                        body.KinematicStepTarget = body.WorldTransform;
                    }
                }
                body.LinearVelocity = Vec3.Zero;
                body.AngularVelocity = Vec3.Zero;
                body.UpdateInertiaWorld();
            }
            World.ClearContacts();
        }

        /// <summary>
        /// FK-rest 物理リセット。剛体を「ボーンの FK-rest ワールド姿勢 * BodyOffsetFromBone」へ置く。
        /// FK-rest = 親駆動のバインド整合姿勢:
        ///   - 物理で動くボーン (動的剛体が紐づくボーン) は、外から与えられた姿勢を使わず、
        ///     親チェーンから前計算する (バインドは位置のみ・回転恒等なので単純な階層合成)。
        ///   - 駆動されるボーン (kinematic 剛体のボーン等) は getDrivenBoneWorld の姿勢を使う。
        /// これにより、CSV(MMDの物理結果=傾き込み)を全剛体へ一斉適用したときの過拘束発散を避け、
        /// BonePoseCsvPlayer / HeadlessDriver / Unity で同一の開始状態を作れる。
        /// getDrivenBoneWorld: ボーンindex → 駆動姿勢 (無ければ null)。物理ボーンには使われない。
        /// </summary>
        public void ResetBodiesToBonePoseFk(System.Func<int, RigidTransform?> getDrivenBoneWorld)
        {
            int n = _model.BoneNames.Count;

            // 物理ボーン = 非 BoneFollow (動的/物理+Bone合わせ) 剛体が紐づくボーン。
            var isPhysics = new bool[n];
            foreach (var link in BoneLinks)
                if (link.BoneIndex >= 0 && link.BoneIndex < n && link.Mode != PhysicsMode.BoneFollow)
                    isPhysics[link.BoneIndex] = true;

            var world = new RigidTransform?[n];
            RigidTransform Fk(int i, int depth)
            {
                if (world[i].HasValue) return world[i].Value;
                // 循環参照の保険 (深さ上限で打ち切りバインド位置)。
                if (depth > 512) { world[i] = new RigidTransform(Quat.Identity, _model.BonePositions[i]); return world[i].Value; }

                // 物理ボーンは駆動姿勢を使わず必ず FK。それ以外は駆動姿勢があれば使う。
                RigidTransform? driven = isPhysics[i] ? null : getDrivenBoneWorld(i);
                RigidTransform res;
                if (driven.HasValue) res = driven.Value;
                else
                {
                    int p = (i < _model.BoneParents.Count) ? _model.BoneParents[i] : -1;
                    if (p < 0 || p >= n)
                        res = new RigidTransform(Quat.Identity, _model.BonePositions[i]); // ルート等 = バインド世界
                    else
                    {
                        var pw = Fk(p, depth + 1);
                        var localOff = _model.BonePositions[i] - _model.BonePositions[p]; // バインドは回転恒等
                        res = new RigidTransform(pw.Rotation, pw.Rotation * localOff + pw.Origin);
                    }
                }
                world[i] = res;
                return res;
            }
            for (int i = 0; i < n; i++) if (!world[i].HasValue) Fk(i, 0);

            // 計算した FK-rest 姿勢で配置 (原始 API へ委譲)。
            ResetBodiesToBonePose(i => (i >= 0 && i < n) ? world[i] : null);
        }

        /// <summary>
        /// 駆動(BoneFollow)剛体の KinematicTarget を「ボーンworld姿勢 * BodyOffsetFromBone」で設定する共通ヘルパ。
        /// ★駆動式は必ずこの1箇所を使うこと。ハーネス/Unityで手書きしない。
        /// (2026-08-09 事故: hairfid が手書きで `= bw`(offset欠落)とし、体コライダーが誤配置のまま
        ///  全simが走って貫入系の数字が全て汚染された。再発防止のため集約+回帰テストあり)
        /// getDrivenBoneWorld が null を返したボーンは前回ターゲット維持 (テレポートしない)。
        /// </summary>
        public void ApplyKinematicTargets(System.Func<int, RigidTransform?> getDrivenBoneWorld)
        {
            foreach (var link in BoneLinks)
            {
                if (link.Mode != PhysicsMode.BoneFollow || link.BoneIndex < 0) continue;
                var bw = getDrivenBoneWorld(link.BoneIndex);
                if (bw.HasValue) link.Body.KinematicTarget = bw.Value * link.BodyOffsetFromBone;
            }
        }

        /// <summary>mode2 (DynamicBoneMerge) の剛体が1つでもあるか。
        /// 書き戻し時に補正姿勢の計算が必要かの判定に使う (無ければ計算ごと省ける)。</summary>
        public bool HasBoneMergeBodies => _hasBoneMerge;

        private bool _hasBoneMerge;

        /// <summary>
        /// [物理+ボーン位置合わせ] 再現 (PmxEditorの補正層。補正OFF/ON対照データで式を確定, 2026-08-09):
        ///   物理ボーンの出力姿勢 = 位置: 親ボーン(補正済)の位置 + 親回転で回した bind オフセット
        ///                          (物理の「移動分」を捨てる) / 回転: 物理回転そのまま。
        /// 検証: |ON子-(ON親+qON親·bindRel)| = skirt中央0.011 (MMD(補正ON)でほぼ厳密成立, OFFは0.072)。
        /// 駆動ボーンは getDrivenBoneWorld、物理ボーンの回転は剛体から復元 (body * offset^-1)。
        /// 書き戻しは HeadlessDriver / MmdPhysicsBehaviour / 計測ハーネスで必ず本ヘルパを共用する
        /// (FK-rest リセットと同じ扱い。経路差のバグを避ける)。戻り値: boneIndex -> 補正済world姿勢。
        /// </summary>
        /// <param name="getDrivenBoneWorld">駆動ボーンの現在world姿勢 (null=バインド)。</param>
        /// <param name="rotClampAlpha">回転clamp割合 (0=無効=回転そのまま / 1=リミットへ完全clamp)。
        ///   [Jointロック内部演算] 再現の第一形: 親側ジョイントの相対euler(補正済親フレーム基準)を
        ///   リミット超過分だけ α で戻す。MMD(補正ON)の超過8-14°は完全clampでないことを示すため中間αを掃引する。</param>
        /// 順序比較(位置を物理回転で再構成→回転のみclamp)は、呼び出し側で α=0 と α>0 の2回呼びを合成する。
        public RigidTransform?[] ComputeAlignedBonePoses(System.Func<int, RigidTransform?> getDrivenBoneWorld,
            float rotClampAlpha = 0f)
        {
            int n = _model.BoneNames.Count;
            var physRot = new Quat?[n]; // 物理ボーンの復元回転 (位置は捨てる)
            var linkOf = new System.Collections.Generic.Dictionary<RigidBody, BoneLink>();
            foreach (var link in BoneLinks)
            {
                if (link.Body != null) linkOf[link.Body] = link;
                if (link.BoneIndex >= 0 && link.BoneIndex < n && link.Mode != PhysicsMode.BoneFollow)
                    physRot[link.BoneIndex] = (link.Body.WorldTransform * link.BodyOffsetFromBone.Inverse()).Rotation;
            }
            // 物理ボーン -> 親側ジョイント (BodyB=自分の剛体)。clamp 用。
            System.Collections.Generic.Dictionary<int, Joint> parentJoint = null;
            if (rotClampAlpha > 0f)
            {
                parentJoint = new System.Collections.Generic.Dictionary<int, Joint>();
                // 階層親(BoneParents)と一致するジョイントを優先 (ring0=取付, ring1/2=縦, 髪=鎖)。
                // 横ジョイント(隣接同士)を誤って親に選ぶと clamp が的を外す (α=1でも取付超過が残った実測の原因)。
                foreach (var j in World.Joints)
                    if (j.BodyB != null && j.BodyA != null && linkOf.ContainsKey(j.BodyB) && linkOf.ContainsKey(j.BodyA))
                    {
                        int bi = linkOf[j.BodyB].BoneIndex;
                        if (bi < 0 || bi >= n || !physRot[bi].HasValue) continue;
                        int hierParent = (bi < _model.BoneParents.Count) ? _model.BoneParents[bi] : -1;
                        bool isHier = linkOf[j.BodyA].BoneIndex == hierParent;
                        if (!parentJoint.ContainsKey(bi)) parentJoint[bi] = j;
                        else if (isHier && linkOf[parentJoint[bi].BodyA].BoneIndex != hierParent) parentJoint[bi] = j;
                    }
            }
            Quat EulerQ(Vec3 e)
            {
                float sx = (float)System.Math.Sin(e.x * 0.5f), cx = (float)System.Math.Cos(e.x * 0.5f);
                float sy = (float)System.Math.Sin(e.y * 0.5f), cy = (float)System.Math.Cos(e.y * 0.5f);
                float sz = (float)System.Math.Sin(e.z * 0.5f), cz = (float)System.Math.Cos(e.z * 0.5f);
                return new Quat(sx, 0, 0, cx) * new Quat(0, sy, 0, cy) * new Quat(0, 0, sz, cz);
            }

            var world = new RigidTransform?[n];
            RigidTransform Align(int i, int depth)
            {
                if (world[i].HasValue) return world[i].Value;
                if (depth > 512) { world[i] = new RigidTransform(Quat.Identity, _model.BonePositions[i]); return world[i].Value; }
                RigidTransform res;
                int p = (i < _model.BoneParents.Count) ? _model.BoneParents[i] : -1;
                if (!physRot[i].HasValue)
                {
                    // 非物理: 駆動姿勢があればそれ、無ければ FK (バインドは回転恒等)。
                    RigidTransform? driven = getDrivenBoneWorld(i);
                    if (driven.HasValue) res = driven.Value;
                    else if (p < 0 || p >= n) res = new RigidTransform(Quat.Identity, _model.BonePositions[i]);
                    else
                    {
                        var pw = Align(p, depth + 1);
                        res = new RigidTransform(pw.Rotation, pw.Rotation * (_model.BonePositions[i] - _model.BonePositions[p]) + pw.Origin);
                    }
                }
                else
                {
                    // 物理: 回転 = 物理 (rotClampAlpha>0 なら親側ジョイントのリミットへ α で戻す)。
                    //        位置 = 親(補正済)から再構成。親無しはバインド位置。
                    var rot = physRot[i].Value;
                    if (parentJoint != null && parentJoint.TryGetValue(i, out var pj))
                    {
                        // 補正済み親フレーム基準の相対euler (エンジンと同じ分解 Joint.ToEulerXYZ)。
                        var aLink = linkOf[pj.BodyA];
                        RigidTransform aBonePose;
                        if (aLink.BoneIndex >= 0 && aLink.BoneIndex < n)
                            aBonePose = Align(aLink.BoneIndex, depth + 1);
                        else
                            aBonePose = pj.BodyA.WorldTransform * aLink.BodyOffsetFromBone.Inverse();
                        var qA = ((aBonePose * aLink.BodyOffsetFromBone).Rotation * pj.FrameInA.Rotation).Normalized;
                        var bLink = linkOf[pj.BodyB];
                        var qB = ((rot * bLink.BodyOffsetFromBone.Rotation) * pj.FrameInB.Rotation).Normalized;
                        var eu = Joint.ToEulerXYZ((qA.Conjugated() * qB).Normalized);
                        var ec = eu; bool changed = false;
                        for (int d = 0; d < 3; d++)
                        {
                            float lo = pj.AngularLowerLimit[d], hi = pj.AngularUpperLimit[d];
                            if (lo > hi) continue; // free
                            if (ec[d] < lo) { ec[d] += rotClampAlpha * (lo - ec[d]); changed = true; }
                            else if (ec[d] > hi) { ec[d] += rotClampAlpha * (hi - ec[d]); changed = true; }
                        }
                        if (changed)
                        {
                            var qBnew = qA * EulerQ(ec);
                            rot = (qBnew * pj.FrameInB.Rotation.Conjugated()) * bLink.BodyOffsetFromBone.Rotation.Conjugated();
                        }
                    }
                    if (p < 0 || p >= n) res = new RigidTransform(rot, _model.BonePositions[i]);
                    else
                    {
                        var pw = Align(p, depth + 1);
                        res = new RigidTransform(rot, pw.Rotation * (_model.BonePositions[i] - _model.BonePositions[p]) + pw.Origin);
                    }
                }
                world[i] = res;
                return res;
            }
            for (int i = 0; i < n; i++) if (!world[i].HasValue) Align(i, 0);
            return world;
        }

        private static RigidTransform ComputeOffset(PmxPhysicsModel model, PmxRigidBody rb)
        {
            var bodyWorld = RigidTransform.FromEuler(rb.Position, rb.Rotation);
            if (rb.BoneIndex < 0 || rb.BoneIndex >= model.BonePositions.Count)
                return bodyWorld; // ボーン無し
            // バインド時ボーンは回転恒等・位置のみ。
            var boneWorld = new RigidTransform(Quat.Identity, model.BonePositions[rb.BoneIndex]);
            return boneWorld.InverseTimes(bodyWorld);
        }

        private static CollisionShape CreateShape(PmxRigidBody rb)
        {
            return rb.ShapeType switch
            {
                0 => new SphereShape(rb.Size.x),
                1 => new BoxShape(rb.Size),
                2 => new CapsuleShape(rb.Size.x, rb.Size.y),
                _ => new SphereShape(rb.Size.x),
            };
        }

        private void BuildJoints(PmxPhysicsModel model)
        {
            foreach (var pj in model.Joints)
            {
                RigidBody a = ValidBody(pj.RigidBodyAIndex);
                RigidBody b = ValidBody(pj.RigidBodyBIndex);
                if (a == null || b == null) continue; // 両端必須

                var worldFrame = RigidTransform.FromEuler(pj.Position, pj.Rotation);
                var joint = Joint.FromPmx(
                    (JointType)pj.JointType, a, b, worldFrame,
                    pj.LinearLowerLimit, pj.LinearUpperLimit,
                    pj.AngularLowerLimit, pj.AngularUpperLimit,
                    pj.SpringLinear, pj.SpringAngular);
                joint.Name = pj.Name;
                World.AddJoint(joint);
            }
        }

        private RigidBody ValidBody(int index) =>
            (index >= 0 && index < Bodies.Count) ? Bodies[index] : null;
    }
}
