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
                body.SetMassProps(body.Mode == PhysicsMode.BoneFollow ? 0f : ClampMass(rb.Mass, rb.Name));

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
        /// <summary>PMX の異常な質量を、float32 のソルバが壊れない範囲へ丸める。
        /// ★2026-08-26 (タスク94)。減衰の [0,0.999] クランプと同じ「異常値への耐性」の一環。
        ///
        /// 背景 (すべて実測):
        ///   あるモデルは 34リンクの鎖に **質量 5.56e14 → 0.1 (毎段 ÷3)** を持つ。
        ///   実機の再生でこの鎖が発散し、画面外まで飛んだ。ヘッドレス再現 (tools/diagnostics/divhunt)
        ///   で切り分けたところ:
        ///     ・接触を止めても**完全に同一** → 接触は無関係。ジョイントソルバ由来
        ///     ・反復 10/20/40・サブステップ 2/4 で**変わらない** → 収束不足ではない
        ///     ・減衰 0.1/0.5/0.9 に変えても**変わらない** → 減衰は無関係
        ///     ・発散の瞬間の入力は **0.03単位 / 1.3度** → 入力は穏やか
        ///     ・**質量に上限を掛けると完全に発散しない**
        ///   上限の掃引: **1e11 までは無事、1e12 から発散**する。float32 の限界域。
        ///
        /// 閾値 1e3 の根拠 (実測で選んだ。単なる余裕取りではない):
        ///   ・参照スイート35モデルの動的質量の最大は **30**。1e3 はその **33倍**で、
        ///     全モデルで一度も発動しない (36モデルスイープはビット同一)。
        ///   ・上限を上げすぎると効かない。**1e6 では静止時の拘束違反が 1.5** と悪化する
        ///     (質量 1e14 で「凍って」いた鎖の拘束が解け、不安定な領域に留まるため)。
        ///     1e3 なら **0.196**、1e2 で 0.204。1e3〜1e2 が実測の最良域。
        ///   ・不安定域 (1e12) からは 9桁下。
        ///   1e14 の剛体を 1e3 に丸めても、鎖の末端 (0.1) との比は 1e4 なので
        ///   「事実上動かない」という作者の意図は保たれる。
        /// ★動的剛体だけが対象。ボーン追従 (mode 0) は元から質量 0 なので影響しない。
        ///
        /// ★Bullet 2.75 にこのクランプは無い。**忠実化ではなく耐性のための逸脱**である。
        ///   参照スイートでは一度も発動しないので、平時の出力はビット不変。
        /// </summary>
        /// ★★2026-08-26 (タスク96) 既定を 0 (無効) へ戻した。**実害が出るまで 0 のまま。**
        ///
        /// ★2026-08-27 タスク84: ここに書いてあった参照比の数値 (定常/スパイクの倍率) は
        ///   **すべて撤回した**。突き合わせに使った参照データが全期間を通じて無効だったため
        ///   (経緯は docs/investigations の該当モデルの参照の件)。
        ///   撤回しない事実だけ残す:
        ///   ・導入時に自前の |v| しきい値だけで採否を判定していたのは誤りだった。
        ///   ・安定性のためにクランプが要るわけではない
        ///     (全身・接触あり・3600F で |p|max 48.7・発散なし。参照非依存の実測)。
        ///   ・実機で「画面外へ飛ぶ」のを止めたのは **起動時テレポートの修正** (タスク93) の方。
        ///   → クランプの要否には現状 **定量的な根拠が無い**。既定 0 を維持する。
        public static float MaxDynamicMass = 0f;   // ★2026-08-26 タスク96: 既定 OFF へ戻した (下記)
        public static int ClampedMassCount;   // 診断用: 何体丸めたか

        private static float ClampMass(float mass, string name)
        {
            if (MaxDynamicMass <= 0f) return mass;          // 0 で無効化 (A/B 用)
            if (!(mass > MaxDynamicMass)) return mass;      // NaN はここを通さない (下の判定へ)
            ClampedMassCount++;
            return MaxDynamicMass;
        }

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
        /// <param name="alignAllPositions">true=物理ボーンの位置を一律で親チェーンから再構成する(従来の挙動)。
        ///   false(既定)=mode1 のボーンは剛体の生の姿勢を返す。書き戻し側 (PullPhysicsToBones) が
        ///   mode1 を生位置で書くため、ここで作り直すと「画面に出る親」と「mode2 の子が基準にした親」が
        ///   食い違う。呼び出し側の AlignBonePositions をそのまま渡すこと。</param>
        public RigidTransform?[] ComputeAlignedBonePoses(System.Func<int, RigidTransform?> getDrivenBoneWorld,
            float rotClampAlpha = 0f, bool alignAllPositions = false)
        {
            int n = _model.BoneNames.Count;
            var physRot = new Quat?[n]; // 物理ボーンの復元回転 (位置は捨てる)
            var linkOf = new System.Collections.Generic.Dictionary<RigidBody, BoneLink>();
            // ボーンindex -> BoneLink。mode1 と mode2 で位置の出どころが違うので Align 側で要る。
            var linkOfBone = new BoneLink?[n];
            foreach (var link in BoneLinks)
            {
                if (link.Body != null) linkOf[link.Body] = link;
                if (link.BoneIndex >= 0 && link.BoneIndex < n && link.Mode != PhysicsMode.BoneFollow)
                {
                    physRot[link.BoneIndex] = (link.Body.WorldTransform * link.BodyOffsetFromBone.Inverse()).Rotation;
                    // ★同じボーンに複数の剛体が付くことがある (例: 腰パーツ親_1/_2/_3)。
                    //   最後に見つけたものが残る。physRot も直上で同じように上書きしているので、
                    //   回転と位置が別の剛体から来ることはない。
                    linkOfBone[link.BoneIndex] = link;
                }
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
                    // ★mode1 (物理演算) のボーンは、書き戻し側が**剛体の生の姿勢をそのまま書く**。
                    //   ここで bind 長の鎖を作り直すと、画面に出る親と、mode2 の子が基準にした親が
                    //   別物になり、鎖の最後の1節だけが伸び縮みする
                    //   (実測: しっぽ１〜１２=mode1 の先の しっぽ１３=mode2 が静止状態で 0.20 倍)。
                    //   MMD の [物理演算+Bone位置合わせ] は「親ボーンの**実際の**現在姿勢」を基準にするので、
                    //   親として返すのも生の姿勢でなければならない。
                    //   alignAllPositions が立っているときだけ、鎖全体を一貫して再構成する。
                    var myLink = linkOfBone[i];
                    if (!alignAllPositions && myLink.HasValue && myLink.Value.Body != null &&
                        myLink.Value.Mode != PhysicsMode.DynamicBoneMerge)
                    {
                        world[i] = myLink.Value.Body.WorldTransform * myLink.Value.BodyOffsetFromBone.Inverse();
                        return world[i].Value;
                    }

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
