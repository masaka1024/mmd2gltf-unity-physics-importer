// ===========================================================================
// Unity Bullet 互換物理エンジン – Narrowphase (GJK + EPA)
// 凸形状同士の接触判定・貫入量・法線・接触点を算出する。
// Bullet の btGjkPairDetector + btGjkEpaPenetrationDepthSolver 相当。
// ===========================================================================

using System;
using System.Collections.Generic;

namespace BulletPhysics
{
    /// <summary>1 つの接触点。ワールド座標系。</summary>
    public struct ContactPoint
    {
        public Vec3 PositionWorldA;   // A 上の接触点
        public Vec3 PositionWorldB;   // B 上の接触点
        public Vec3 LocalPointA;      // A ローカルの接触点 (Refresh 再投影用)
        public Vec3 LocalPointB;      // B ローカルの接触点 (Refresh 再投影用)
        public Vec3 Normal;           // B から A へ向かう単位法線
        public float Distance;        // 負値 = 貫入量

        /// <summary>★タスク68: このフレームのナローフェーズが「この点はまだ在る」と
        /// 確認したか。Refresh の頭で false に落とし、AddPoint が
        /// (既存に一致して置換 / 新規に追加) したときだけ true になる。
        /// 幻の定義そのもの (= 新点で確認されなかった古い点) を表すので、
        /// 深い側の破棄をこのフラグが false の点だけに限定できる。</summary>
        public bool ConfirmedThisStep;

        // ソルバ用の蓄積インパルス (ウォームスタート)。
        public float NormalImpulse;
        public float TangentImpulse1;
        public float TangentImpulse2;
    }

    /// <summary>剛体ペアの接触点集合 (最大 4 点)。フレーム間で保持する。</summary>
    public sealed class PersistentManifold
    {
        public RigidBody BodyA;
        public RigidBody BodyB;
        public readonly List<ContactPoint> Points = new(4);

        public PersistentManifold(RigidBody a, RigidBody b) { BodyA = a; BodyB = b; }

        /// <summary>タスク51: 接触点の管理 (同一判定・4点超過時の置換・破棄) を
        /// Bullet 2.75 btPersistentManifold の実ソースへ揃える (既定 false = ビット不変)。
        ///
        /// 実測 (スカート網・60秒・両側 SubSteps=2) で支持面の育ち方が決定的に違った:
        ///   59秒の点数分布  自前 1点=8 / 2点=10 / 3点=2 / 4点=1   総点数 38
        ///                   Bullet 1点=7 / 2点=0 / 3点=2 / 4点=14  総点数 69
        ///   per-frame の変位比は 60秒通して 32倍 のまま平行
        ///   (Bullet は1秒で収束、当エンジンは一度も落ちない)。
        /// 支持面が育たない・安定しないので、スカートが載り切らずに揺れ続けていた。
        ///
        /// 揃える規則は3つ。**部品化しない** (Bullet 側で一体の仕組みなので):
        ///   1. 同一判定  getCacheEntry   : LocalPointA の距離が閾値の2乗未満なら同じ点として置換。
        ///                従来は PositionWorldA の距離 0.02 固定で、広すぎて点が育たなかった。
        ///   2. 4点超過時 sortCachedPoints: 4通りの組の面積 (外積長の2乗) を計算して最大を残す。
        ///                加えて KEEP_DEEPEST_POINT で最深点は置換候補から外す。
        ///                従来は「最も浅い点を置換」だけで支持が偏っていた。
        ///   3. 破棄      refreshContactPoints: 法線距離 > 閾値、または法線へ射影した残差の
        ///                長さの2乗 > 閾値の2乗 で破棄。従来は 0.04 固定で横ずれの取り方も別式。
        /// 閾値は形状サイズ比例の接触破棄閾値を使う。当てる値は作らない。</summary>
        public static bool BulletManifoldPoints = true;   // ★完全セットv1 で既定ON (2026-08-23)

        /// <summary>診断カウンタ (タスク54)。ON の間だけ数える。数えるだけなのでビット不変。
        /// warm-start が切れる箇所が Refresh の破棄か 同一判定の不成立かを分けるために足した。</summary>
        public static bool CollectManifoldStats = false;
        public static long StatRefreshKept, StatRefreshDropNormal, StatRefreshDropLateral, StatRefreshDropDeep;
        public static long StatAddMatched, StatAddNewSlot, StatAddReplaced;
        public static void ResetManifoldStats()
        {
            StatRefreshKept = StatRefreshDropNormal = StatRefreshDropLateral = StatRefreshDropDeep = 0;
            StatAddMatched = StatAddNewSlot = StatAddReplaced = 0;
        }

        /// <summary>このペアの接触破棄閾値。Bullet の getContactBreakingThreshold() 相当。</summary>
        private float BreakingThreshold =>
            Math.Min(GjkEpa.BulletBreakingThresholdOf(BodyA.Shape),
                     GjkEpa.BulletBreakingThresholdOf(BodyB.Shape));

        /// <summary>★タスク67: validContactDistance を **深い側にも対称に**適用する
        /// (既定 false = ビット不変)。閾値は既存の contactBreakingThreshold をそのまま使い、
        /// 新しいパラメータは足さない。
        ///
        /// ★これは Bullet に無い機構である。2.75 も現行 3.x master も
        ///   `validContactDistance(pt) { return pt.m_distance1 <= getContactBreakingThreshold(); }`
        ///   の**片側**判定のままで、深い側の番人はどこにも無い
        ///   (bulletphysics/bullet3 master btPersistentManifold.h / .cpp を 2026-08-23 に確認)。
        ///   したがって北極星 原則2 の「登録済み逸脱 + 出力ゲート判定」の手続きで扱う。
        ///
        /// なぜ要るか (タスク65/66 の実測):
        ///   剛体が回ると、保存済みローカル点は真の最近接から滑る。
        ///   `d = (worldA - worldB)·normal` は法線が凍結したまま **負へ暴走**し、
        ///   実測で真値「接触なし」のペアが d = -0.66 まで育った。
        ///   片側判定は正の側しか見ず、横ずれ判定は法線方向のドリフトを捕まえないので、
        ///   この点は誰にも殺されない。しかも新しく生成された正しい点は
        ///   getCacheEntry の一致閾値 (0.0354) に対し pA が 0.50〜1.10 も離れているため
        ///   **別スロットに入るだけで幻を置換できない**。
        ///   幻の d はそのまま接触 rhs に載る: NormalBias = -d*erp/dt = 0.2*0.66*60 = 7.9。
        ///   定常窓の接触行の 14% (22/154) がこの幻だった。
        ///
        /// ★タスク68 で **鮮度条件つき** に限定した。破棄の対象は
        ///   「そのフレームのナローフェーズに確認されなかった点」だけ (ContactPoint.ConfirmedThisStep)。
        ///   タスク67 の無条件版は正当に深い接触まで毎フレーム捨てて warm-start を切ってしまい、
        ///   スカート (接触点の **61%** が閾値 0.0109 より深い) を 0.73x -> 0.17x と過減衰させた。
        ///   鮮度条件を入れると、毎フレーム作り直される本物の深い接触は無傷のまま
        ///   滑って取り残された幻だけを殺せる。</summary>
        public static bool SymmetricBreakingDistance = true;   // ★完全セットv1 で既定ON (2026-08-23)

        // btPersistentManifold::refreshContactPoints の移植。
        private void RefreshBullet()
        {
            float thr = BreakingThreshold;
            float thr2 = thr * thr;
            for (int i = Points.Count - 1; i >= 0; i--)
            {
                var cp = Points[i];
                var worldA = BodyA.WorldTransform.TransformPoint(cp.LocalPointA);
                var worldB = BodyB.WorldTransform.TransformPoint(cp.LocalPointB);
                float d = (worldA - worldB).Dot(cp.Normal);
                cp.PositionWorldA = worldA;
                cp.PositionWorldB = worldB;
                cp.Distance = d;
                // validContactDistance: 法線方向に閾値を超えて離れたら破棄。
                if (d > thr) { if (CollectManifoldStats) StatRefreshDropNormal++; Points.RemoveAt(i); continue; }
                // 法線成分を除いた残差 (Bullet と同じ射影の取り方) が閾値の2乗を超えたら破棄。
                var projected = worldA - cp.Normal * d;
                var diff2 = worldB - projected;
                if (diff2.LengthSquared > thr2) { if (CollectManifoldStats) StatRefreshDropLateral++; Points.RemoveAt(i); continue; }
                if (CollectManifoldStats) StatRefreshKept++;
                cp.ConfirmedThisStep = false;   // ★タスク68: 今フレームの確認はまだ無い
                Points[i] = cp;
            }
        }

        /// <summary>★タスク68: 深い側の破棄。**Refresh → Detect → AddPoint のあとに** 呼ぶ。
        /// このフレームのナローフェーズに確認されなかった点だけを対象にするので、
        /// 毎フレーム作り直される正当な深い接触 (スカートは接触点の61%が閾値より深い) は無傷。
        /// 閾値は既存の contactBreakingThreshold をそのまま流用する。</summary>
        public void PruneStaleDeep()
        {
            if (!SymmetricBreakingDistance) return;
            float thr = BulletManifoldPoints ? BreakingThreshold : 0.04f;
            for (int i = Points.Count - 1; i >= 0; i--)
            {
                var cp = Points[i];
                if (cp.ConfirmedThisStep || cp.Distance >= -thr) continue;
                if (CollectManifoldStats) StatRefreshDropDeep++;
                Points.RemoveAt(i);
            }
        }

        // btPersistentManifold の addManifoldPoint / getCacheEntry の移植。
        private void AddPointBullet(ContactPoint cp)
        {
            float shortest = BreakingThreshold * BreakingThreshold;
            int near = -1;
            for (int i = 0; i < Points.Count; i++)
            {
                float d2 = (Points[i].LocalPointA - cp.LocalPointA).LengthSquared;
                if (d2 < shortest) { shortest = d2; near = i; }
            }
            cp.ConfirmedThisStep = true;   // ★タスク68: 今フレームのナローフェーズ由来
            if (near >= 0)
            {
                if (CollectManifoldStats) StatAddMatched++;
                cp.NormalImpulse = Points[near].NormalImpulse;
                cp.TangentImpulse1 = Points[near].TangentImpulse1;
                cp.TangentImpulse2 = Points[near].TangentImpulse2;
                Points[near] = cp;
                return;
            }
            if (Points.Count < 4) { if (CollectManifoldStats) StatAddNewSlot++; Points.Add(cp); return; }
            if (CollectManifoldStats) StatAddReplaced++;
            Points[SortCachedPoints(cp)] = cp;
        }

        // btPersistentManifold::sortCachedPoints の移植。置換すべき index を返す。
        //   4通りの「新点を入れて i を捨てた組」の面積 (外積長の2乗) を出し、最大の組を選ぶ。
        //   KEEP_DEEPEST_POINT: 最深点はその case の面積を 0 のままにして候補から外す。
        private int SortCachedPoints(ContactPoint pt)
        {
            int maxPenetrationIndex = -1;
            float maxPenetration = pt.Distance;
            for (int i = 0; i < 4; i++)
                if (Points[i].Distance < maxPenetration)
                { maxPenetrationIndex = i; maxPenetration = Points[i].Distance; }

            float res0 = 0f, res1 = 0f, res2 = 0f, res3 = 0f;
            if (maxPenetrationIndex != 0)
                res0 = Vec3.Cross(pt.LocalPointA - Points[1].LocalPointA,
                                  Points[3].LocalPointA - Points[2].LocalPointA).LengthSquared;
            if (maxPenetrationIndex != 1)
                res1 = Vec3.Cross(pt.LocalPointA - Points[0].LocalPointA,
                                  Points[3].LocalPointA - Points[2].LocalPointA).LengthSquared;
            if (maxPenetrationIndex != 2)
                res2 = Vec3.Cross(pt.LocalPointA - Points[0].LocalPointA,
                                  Points[3].LocalPointA - Points[1].LocalPointA).LengthSquared;
            if (maxPenetrationIndex != 3)
                res3 = Vec3.Cross(pt.LocalPointA - Points[0].LocalPointA,
                                  Points[2].LocalPointA - Points[1].LocalPointA).LengthSquared;

            // btVector4::closestAxis4 = 最大成分の index。
            int best = 0; float bv = res0;
            if (res1 > bv) { bv = res1; best = 1; }
            if (res2 > bv) { bv = res2; best = 2; }
            if (res3 > bv) { bv = res3; best = 3; }
            return best;
        }

        public void Refresh()
        {
            if (BulletManifoldPoints) { RefreshBullet(); return; }
            // 各接触点をローカル座標から現在姿勢でワールドへ再投影し、
            // 法線方向に離れた/横ずれした点を破棄する。生存点は位置を更新。
            for (int i = Points.Count - 1; i >= 0; i--)
            {
                var cp = Points[i];
                var worldA = BodyA.WorldTransform.TransformPoint(cp.LocalPointA);
                var worldB = BodyB.WorldTransform.TransformPoint(cp.LocalPointB);
                var diff = worldA - worldB;
                var d = diff.Dot(cp.Normal);
                var lateral = diff - cp.Normal * d;
                if (d > 0.04f || lateral.LengthSquared > 0.04f * 0.04f)
                {
                    if (CollectManifoldStats)
                    { if (d > 0.04f) StatRefreshDropNormal++; else StatRefreshDropLateral++; }
                    Points.RemoveAt(i);
                    continue;
                }
                if (CollectManifoldStats) StatRefreshKept++;
                // 生存: 再投影した位置と貫入量をソルバへ渡すため更新。
                cp.PositionWorldA = worldA;
                cp.PositionWorldB = worldB;
                cp.Distance = d;
                cp.ConfirmedThisStep = false;   // ★タスク68
                Points[i] = cp;
            }
        }

        public void AddPoint(ContactPoint cp)
        {
            if (BulletManifoldPoints) { AddPointBullet(cp); return; }
            cp.ConfirmedThisStep = true;   // ★タスク68
            // 近い既存点があればウォームスタート値を引き継いで置換。
            const float mergeDist2 = 0.02f * 0.02f;
            for (int i = 0; i < Points.Count; i++)
            {
                if ((Points[i].PositionWorldA - cp.PositionWorldA).LengthSquared < mergeDist2)
                {
                    if (CollectManifoldStats) StatAddMatched++;
                    cp.NormalImpulse = Points[i].NormalImpulse;
                    cp.TangentImpulse1 = Points[i].TangentImpulse1;
                    cp.TangentImpulse2 = Points[i].TangentImpulse2;
                    Points[i] = cp;
                    return;
                }
            }
            if (Points.Count < 4) { if (CollectManifoldStats) StatAddNewSlot++; Points.Add(cp); }
            else { if (CollectManifoldStats) StatAddReplaced++; Points[WorstPointIndex(cp)] = cp; }
        }

        private int WorstPointIndex(ContactPoint candidate)
        {
            // 最も浅い点を置換候補にする簡易版。
            int worst = 0; float maxDist = Points[0].Distance;
            for (int i = 1; i < Points.Count; i++)
                if (Points[i].Distance > maxDist) { maxDist = Points[i].Distance; worst = i; }
            return worst;
        }
    }

    /// <summary>GJK による距離判定と EPA による貫入解決。</summary>
    public static class GjkEpa
    {
        private const int MaxIterations = 32;
        private const float Epsilon = 1e-7f;

        private struct SupportVert
        {
            public Vec3 V;   // Minkowski 差の点
            public Vec3 A;   // A 上の support
            public Vec3 B;   // B 上の support
        }

        private static Vec3 WorldSupport(RigidBody body, Vec3 dirWorld, out Vec3 witness)
        {
            var localDir = body.WorldTransform.InverseTransformDirection(dirWorld);
            var localP = body.Shape.LocalSupportWithMargin(localDir);
            witness = body.WorldTransform.TransformPoint(localP);
            return witness;
        }

        private static SupportVert Support(RigidBody a, RigidBody b, Vec3 dir)
        {
            var pa = WorldSupport(a, dir, out var wa);
            var pb = WorldSupport(b, -dir, out var wb);
            return new SupportVert { V = pa - pb, A = wa, B = wb };
        }

        // 縮退ガード用の小さな閾値。
        private const float ContactEps = 1e-9f;

        // 投機的接触マージン。表面が触れる少し手前から接触を生成して連続化し、
        // 貫入 on/off の振動 (Baumgarte のエネルギー注入) を防ぐ。Bullet の
        // collision margin と同じ役割。形状の Radius マージンとは別物 (二重計上しない)。
        // ★2026-08-13: A/B 用に const → static field 化。既定 0.02 は従来値なのでビット不変。
        //   貫入調査で判明: この帯は「速度を見ない固定距離」で、駆動剛体は接触点で 1/60 あたり
        //   法線方向へ中央 0.026 動く = 中央値ですら 1ステップで帯を越える。その結果、貫入の 58% は
        //   「前フレームに接触点ゼロ」から始まり、生成時点で既に深い。
        // ★★測定結果: 帯を広げても貫入は直らない (2026-08-13, モデルA 1200フレーム, SubSteps=2)。
        //   0.02 → 0.15 で「帯を飛び越えた割合」は 58.3% → 7.8% と狙いどおり激減するのに、
        //   貫入中央は 0.0786 → 0.0812 で**まったく動かない**。0.08 では深貫入>0.5 が 5→20件と悪化。
        //   = 律速は「検出が遅い」ことではなく「1ステップで押し出しきれない」ことだった。
        //   → 速度依存マージン (Bullet 2.8x / Box2D の speculative contact) は**不採用**。
        //   効いたのは刻みを細かくする方 (SubSteps 2→4 で貫入中央 -45%)。dt が小さいほど
        //   Baumgarte の補正速度 (factor*pen/dt) が大きくなり、押し出し回数も増えるため。
        //   このつまみは負の結果の再現用に残す。既定 0.02 から動かさないこと。
        //   ※広げるときは PhysicsWorld のブロードフェーズ AABB も同じだけ広げないと、
        //     ペアが AABB 段階で捨てられて効かない (PhysicsWorld.BroadphaseNarrowphase 参照)。
        public static float SpeculativeMargin = 0.02f;
        public const float SpeculativeMarginDefault = 0.02f;

        // ★タスク38: 接触の受理閾値を Bullet 2.75 の方式へ (既定 false = ビット不変)。
        //   当エンジン: 全ペア一律 SpeculativeMargin = 0.02 の**固定距離**。
        //   Bullet 2.75: ペアごとに **形状サイズ比例**。
        //     btCollisionDispatcher.cpp:84
        //       threshold = min(shapeA->getContactBreakingThreshold(), shapeB->...)
        //     btCollisionShape.cpp:48
        //       getContactBreakingThreshold() = getAngularMotionDisc() * gContactThresholdFactor
        //       gContactThresholdFactor = 0.02 (btCollisionShape.cpp:18)
        //     btCollisionShape.cpp:52
        //       getAngularMotionDisc() = disc + |center|
        //       (getBoundingSphere は**単位変換**の getAabb から
        //        disc = |max-min|*0.5 / center = (min+max)*0.5 を作る)
        //   PMX の3形状はどれもローカル原点対称なので center = 0、disc = ローカルAABBの半対角。
        //   実測の差: 半幅(0.347,0.371,0.200)のスカート箱で Bullet 0.0109 に対し当方 0.02。
        //   **当方が約2倍広く拾っている** = 分離しているペアまで接触に上げている。
        public static bool BulletContactThreshold = true;   // ★完全セットv1 で既定ON (2026-08-23)

        /// <summary>Bullet 2.75 gContactThresholdFactor。つまみにしない (実ソースの値)。</summary>
        public const float ContactThresholdFactor = 0.02f;

        // このペアで使う受理閾値。Detect の冒頭で毎回決める。
        //   エンジンは単一スレッドで回す前提 (PhysicsWorld のループは逐次)。
        private static float _pairThreshold = SpeculativeMarginDefault;

        /// <summary>Bullet 2.75 btCollisionShape::getContactBreakingThreshold() 相当。
        /// ローカル AABB は **Bullet の getAabb と同じ取り方** をする:
        ///   球     : (r,r,r)                    … margin = r なので実質そのまま
        ///   箱     : HalfExtents                … getHalfExtentsWithMargin() と一致
        ///   カプセル: (r,hh+r,r) + margin(0.04) … btCapsuleShape::getAabb が margin を足すため</summary>
        /// <summary>ペアの片側ぶんの接触破棄閾値。PersistentManifold からも使う。</summary>
        internal static float BulletBreakingThresholdOf(CollisionShape s) => BulletBreakingThreshold(s);

        private static float BulletBreakingThreshold(CollisionShape s)
        {
            Vec3 h;
            if (s is SphereShape sp) h = new Vec3(sp.Radius, sp.Radius, sp.Radius);
            else if (s is BoxShape bx) h = bx.HalfExtents;
            else if (s is CapsuleShape cp)
            {
                float m = CollisionShape.BulletConvexDistanceMargin;   // 0.04
                h = new Vec3(cp.Radius + m, cp.HalfHeight + cp.Radius + m, cp.Radius + m);
            }
            else { float r = s.BoundingRadius; h = new Vec3(r, r, r); }
            return h.Length * ContactThresholdFactor;   // disc(=|h|) * 0.02、center=0
        }

        /// <summary>この形状が片側として持つ接触**受理**閾値。Detect はペアの両側の min を使う。
        /// PhysicsWorld のブロードフェーズ早期棄却 (Issue #2) が「Detect が受理し得る最大距離」を
        /// 見積もるのに使うので、Detect の閾値決定と**必ず同じ式**であること (ここ1箇所に集約)。</summary>
        public static float AcceptThresholdOf(CollisionShape s) =>
            BulletContactThreshold ? BulletBreakingThreshold(s) : SpeculativeMargin;

        // カプセル軸がほぼ平行とみなす閾値 (sin^2θ)。
        // cross(dA,dB)^2 = |dA|^2|dB|^2 sin^2θ なので、正規化した外積長^2 と比較する。
        // 1e-3 は sinθ≈0.0316 (≈1.8°)。スカート等の面接触が数度以内で平行判定されるよう
        // やや緩めに設定 (2点接触にして転がりを防ぐのが目的)。
        private const float CapsuleParallelSinSq = 1e-3f;

        /// <summary>
        /// A と B の接触を判定し、接触点を outPoints へ格納する (0..複数)。
        /// MMD の球/箱/カプセルは可能な限り解析解で解き、残りは GJK+EPA にフォールバックする。
        /// 法線は A→B、Distance は貫入で負 (既存規約)。
        /// </summary>
        public static void Detect(RigidBody a, RigidBody b, List<ContactPoint> outPoints)
        {
            // ★このペアの受理閾値を先に決める (タスク38)。既定は従来どおり固定 0.02。
            //   (BulletContactThreshold=false なら両側とも SpeculativeMargin で min は恒等 = 従来と同値)
            _pairThreshold = Math.Min(AcceptThresholdOf(a.Shape), AcceptThresholdOf(b.Shape));

            var ta = a.Shape.Type;
            var tb = b.Shape.Type;

            if (ta == ShapeType.Sphere && tb == ShapeType.Sphere)
                SphereSphere(a, b, outPoints);
            else if (ta == ShapeType.Sphere && tb == ShapeType.Capsule)
                SphereCapsule(a, b, sphereIsA: true, outPoints);
            else if (ta == ShapeType.Capsule && tb == ShapeType.Sphere)
                SphereCapsule(b, a, sphereIsA: false, outPoints);
            else if (ta == ShapeType.Capsule && tb == ShapeType.Capsule)
                CapsuleCapsule(a, b, outPoints);
            else if (ta == ShapeType.Sphere && tb == ShapeType.Box)
                SphereBox(a, b, sphereIsA: true, outPoints);
            else if (ta == ShapeType.Box && tb == ShapeType.Sphere)
                SphereBox(b, a, sphereIsA: false, outPoints);
            else if (ta == ShapeType.Capsule && tb == ShapeType.Box)
                CapsuleBox(a, b, capsuleIsA: true, outPoints);
            else if (ta == ShapeType.Box && tb == ShapeType.Capsule)
                CapsuleBox(b, a, capsuleIsA: false, outPoints);
            else
            {
                // 箱×箱 のみ GJK+EPA (タスクBの安全弁で保護)。
                // カプセル×箱は解析化済み: 薄い箱(スカート厚み0.085等)での EPA 縮退による
                // 接触取りこぼし=脚カプセル貫通を避けるため。
                if (GjkEpaPenetration(a, b, out var cp))
                    outPoints.Add(cp);
            }
        }

        // 接触点を A→B 規約で outPoints へ追加。LocalPoint も必ず埋める。
        private static void Emit(RigidBody A, RigidBody B, List<ContactPoint> outPoints,
            Vec3 normalAtoB, float sep, Vec3 pA, Vec3 pB)
        {
            var cp = new ContactPoint
            {
                Normal = normalAtoB,
                Distance = sep,
                PositionWorldA = pA,
                PositionWorldB = pB,
                LocalPointA = A.WorldTransform.InverseTransformPoint(pA),
                LocalPointB = B.WorldTransform.InverseTransformPoint(pB),
            };
            outPoints.Add(cp);
        }

        // --- 球×球 ---
        private static void SphereSphere(RigidBody a, RigidBody b, List<ContactPoint> outPoints)
        {
            var cA = a.WorldTransform.Origin; float rA = ((SphereShape)a.Shape).Radius;
            var cB = b.WorldTransform.Origin; float rB = ((SphereShape)b.Shape).Radius;
            var dab = cB - cA; float rsum = rA + rB;
            float dist2 = dab.LengthSquared;
            float rlim = rsum + _pairThreshold;
            if (dist2 >= rlim * rlim) return;

            float dist = (float)Math.Sqrt(dist2);
            var n = dist > ContactEps ? dab / dist : Vec3.YAxis; // A→B、中心一致は退避
            float sep = dist - rsum;
            Emit(a, b, outPoints, n, sep, cA + n * rA, cB - n * rB);
        }

        // --- 球×カプセル (sphere, capsule はどちらが A/B かを sphereIsA で指定) ---
        private static void SphereCapsule(RigidBody sphere, RigidBody capsule,
            bool sphereIsA, List<ContactPoint> outPoints)
        {
            var sc = sphere.WorldTransform.Origin; float sr = ((SphereShape)sphere.Shape).Radius;
            CapsuleSegment(capsule, out var q0, out var q1, out float cr);

            var cc = ClosestPtPointSegment(sc, q0, q1);
            var d = cc - sc; float rsum = sr + cr;
            float dist = d.Length;
            if (dist >= rsum + _pairThreshold) return;

            var nSphereToCap = dist > ContactEps ? d / dist : Vec3.YAxis;
            float sep = dist - rsum;
            var pSphere = sc + nSphereToCap * sr;
            var pCap = cc - nSphereToCap * cr;

            if (sphereIsA)
                Emit(sphere, capsule, outPoints, nSphereToCap, sep, pSphere, pCap);
            else
                Emit(capsule, sphere, outPoints, -nSphereToCap, sep, pCap, pSphere);
        }

        // --- カプセル×カプセル (平行時は2点) ---
        private static void CapsuleCapsule(RigidBody a, RigidBody b, List<ContactPoint> outPoints)
        {
            CapsuleSegment(a, out var a0, out var a1, out float rA);
            CapsuleSegment(b, out var b0, out var b1, out float rB);
            var dA = a1 - a0; var dB = b1 - b0;
            float lenA2 = dA.LengthSquared, lenB2 = dB.LengthSquared;
            float rsum = rA + rB;

            // 平行判定 → 重なり区間の両端で2点接触。
            var cross = Vec3.Cross(dA, dB);
            bool parallel = lenA2 > ContactEps && lenB2 > ContactEps &&
                            cross.LengthSquared <= CapsuleParallelSinSq * lenA2 * lenB2;
            if (parallel)
            {
                int before = outPoints.Count;
                float LA = (float)Math.Sqrt(lenA2);
                var dAn = dA / LA;
                float tb0 = (b0 - a0).Dot(dAn);
                float tb1 = (b1 - a0).Dot(dAn);
                float lo = Math.Max(0f, Math.Min(tb0, tb1));
                float hi = Math.Min(LA, Math.Max(tb0, tb1));
                if (hi >= lo)
                {
                    EmitParallelPoint(a, b, outPoints, a0, dAn, lo, b0, b1, rA, rB, rsum);
                    if (hi > lo + ContactEps)
                        EmitParallelPoint(a, b, outPoints, a0, dAn, hi, b0, b1, rA, rB, rsum);
                    if (outPoints.Count > before) return; // 2点 (または1点) 生成できた
                }
                // 区間が無い/貫入なし → 単一最近点へフォールバック。
            }

            // 単一最近点。
            ClosestPtSegmentSegment(a0, a1, b0, b1, out _, out _, out var c1, out var c2);
            var d = c2 - c1; float dist = d.Length;
            if (dist >= rsum + _pairThreshold) return;
            Vec3 n = dist > ContactEps ? d / dist : PerpVector(dA); // A→B、縮退は軸垂直
            float sep = dist - rsum;
            Emit(a, b, outPoints, n, sep, c1 + n * rA, c2 - n * rB);
        }

        // 平行カプセルの A軸パラメータ param における接触点を (貫入していれば) 追加。
        private static void EmitParallelPoint(RigidBody a, RigidBody b, List<ContactPoint> outPoints,
            Vec3 a0, Vec3 dAn, float param, Vec3 b0, Vec3 b1, float rA, float rB, float rsum)
        {
            var cA = a0 + dAn * param;
            var cB = ClosestPtPointSegment(cA, b0, b1);
            var d = cB - cA; float dist = d.Length;
            if (dist >= rsum + _pairThreshold) return;
            var n = dist > ContactEps ? d / dist : PerpVector(dAn);
            float sep = dist - rsum;
            Emit(a, b, outPoints, n, sep, cA + n * rA, cB - n * rB);
        }

        // 球中心(箱ローカル座標 local)・箱半サイズ he・実効半径 r から、
        // 「箱→球」ローカル法線・分離(sep<0 で貫入)・箱表面点(ローカル)を解く。
        // 非接触(受理閾値 _pairThreshold 超過)なら false。SphereBox / CapsuleBox の共通コア。
        private static bool SolveSphereBoxLocal(Vec3 local, Vec3 he, float r,
            out Vec3 nLocalBoxToSphere, out float sep, out Vec3 boxSurfLocal)
        {
            bool inside = Math.Abs(local.x) <= he.x &&
                          Math.Abs(local.y) <= he.y &&
                          Math.Abs(local.z) <= he.z;
            if (!inside)
            {
                var q = new Vec3(
                    Math.Clamp(local.x, -he.x, he.x),
                    Math.Clamp(local.y, -he.y, he.y),
                    Math.Clamp(local.z, -he.z, he.z));
                var dl = local - q; float dist = dl.Length;
                if (dist >= r + _pairThreshold)
                {
                    nLocalBoxToSphere = Vec3.YAxis; sep = 0f; boxSurfLocal = q;
                    return false;
                }
                nLocalBoxToSphere = dist > ContactEps ? dl / dist : Vec3.YAxis;
                boxSurfLocal = q;
                sep = dist - r;
                return true;
            }
            // 中心が箱内部 → 最も近い面へ押し出す。
            float best = float.MaxValue; int axis = 0; float sign = 1f;
            for (int i = 0; i < 3; i++)
            {
                float toPos = he[i] - local[i];
                float toNeg = local[i] + he[i];
                if (toPos < best) { best = toPos; axis = i; sign = +1f; }
                if (toNeg < best) { best = toNeg; axis = i; sign = -1f; }
            }
            var nl = Vec3.Zero; nl[axis] = sign;
            nLocalBoxToSphere = nl;
            boxSurfLocal = local; boxSurfLocal[axis] = sign * he[axis];
            sep = -(best + r); // 貫入深さ = 面までの距離 + 半径
            return true;
        }

        // --- 球×箱 (sphere, box を sphereIsA で指定) ---
        private static void SphereBox(RigidBody sphere, RigidBody box,
            bool sphereIsA, List<ContactPoint> outPoints)
        {
            var sc = sphere.WorldTransform.Origin; float sr = ((SphereShape)sphere.Shape).Radius;
            var he = ((BoxShape)box.Shape).HalfExtents;
            var bt = box.WorldTransform;

            var local = bt.InverseTransformPoint(sc);
            if (!SolveSphereBoxLocal(local, he, sr, out var nLocalBoxToSphere, out var sep, out var boxSurfLocal))
                return;

            var nWorldBoxToSphere = bt.TransformDirection(nLocalBoxToSphere).Normalized;
            var boxSurfWorld = bt.TransformPoint(boxSurfLocal);
            var sphereSurfWorld = sc - nWorldBoxToSphere * sr;

            if (sphereIsA)
                Emit(sphere, box, outPoints, -nWorldBoxToSphere, sep, sphereSurfWorld, boxSurfWorld);
            else
                Emit(box, sphere, outPoints, nWorldBoxToSphere, sep, boxSurfWorld, sphereSurfWorld);
        }

        // --- カプセル×箱 (capsule, box を capsuleIsA で指定) ---
        // カプセル線分と箱(OBB)の最近点対を交互射影で近似し、その線分上の点を
        // 「球中心」とみなして球×箱コアで解く。薄い箱でも EPA の縮退を避け、
        // 解析的に安定した接触(法線・貫入量)を与える。
        private static void CapsuleBox(RigidBody capsule, RigidBody box,
            bool capsuleIsA, List<ContactPoint> outPoints)
        {
            CapsuleSegment(capsule, out var p0, out var p1, out float cr);
            var he = ((BoxShape)box.Shape).HalfExtents;
            var bt = box.WorldTransform;

            // 箱ローカル空間へ (OBB → 原点中心 AABB[-he,he])。距離は剛体変換で不変。
            var q0 = bt.InverseTransformPoint(p0);
            var q1 = bt.InverseTransformPoint(p1);

            // 線分[q0,q1] と AABB の最近点対を交互射影で求める (凸集合間の最近点)。
            var boxPt = new Vec3(
                Math.Clamp((q0.x + q1.x) * 0.5f, -he.x, he.x),
                Math.Clamp((q0.y + q1.y) * 0.5f, -he.y, he.y),
                Math.Clamp((q0.z + q1.z) * 0.5f, -he.z, he.z));
            var segPt = boxPt;
            for (int k = 0; k < 8; k++)
            {
                segPt = ClosestPtPointSegment(boxPt, q0, q1);
                var np = new Vec3(
                    Math.Clamp(segPt.x, -he.x, he.x),
                    Math.Clamp(segPt.y, -he.y, he.y),
                    Math.Clamp(segPt.z, -he.z, he.z));
                if ((np - boxPt).LengthSquared <= 1e-12f) { boxPt = np; break; }
                boxPt = np;
            }

            // 線分上の最近点(ローカル)を球中心として球×箱で解く。
            if (!SolveSphereBoxLocal(segPt, he, cr, out var nLocalBoxToCap, out var sep, out var boxSurfLocal))
                return;

            var nWorldBoxToCap = bt.TransformDirection(nLocalBoxToCap).Normalized;
            var boxSurfWorld = bt.TransformPoint(boxSurfLocal);
            var capSurfWorld = bt.TransformPoint(segPt) - nWorldBoxToCap * cr;

            if (capsuleIsA)
                Emit(capsule, box, outPoints, -nWorldBoxToCap, sep, capSurfWorld, boxSurfWorld);
            else
                Emit(box, capsule, outPoints, nWorldBoxToCap, sep, boxSurfWorld, capSurfWorld);
        }

        // --- 幾何ヘルパー ---

        // カプセルの線分端点 (ワールド) と半径。マージン機構は使わず素の幾何値を使う。
        private static void CapsuleSegment(RigidBody body, out Vec3 p0, out Vec3 p1, out float radius)
        {
            var cap = (CapsuleShape)body.Shape;
            radius = cap.Radius;
            float hh = cap.HalfHeight;
            p0 = body.WorldTransform.TransformPoint(new Vec3(0, hh, 0));
            p1 = body.WorldTransform.TransformPoint(new Vec3(0, -hh, 0));
        }

        private static Vec3 ClosestPtPointSegment(Vec3 p, Vec3 a, Vec3 b)
        {
            var ab = b - a;
            float denom = ab.LengthSquared;
            if (denom < ContactEps) return a; // 縮退線分
            float t = (p - a).Dot(ab) / denom;
            t = Math.Clamp(t, 0f, 1f);
            return a + ab * t;
        }

        // Ericson "Real-Time Collision Detection" ClosestPtSegmentSegment 相当。
        private static void ClosestPtSegmentSegment(
            Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2,
            out float s, out float t, out Vec3 c1, out Vec3 c2)
        {
            var d1 = q1 - p1; var d2 = q2 - p2; var r = p1 - p2;
            float a = d1.LengthSquared, e = d2.LengthSquared, f = d2.Dot(r);

            if (a <= ContactEps && e <= ContactEps)
            {
                s = t = 0f; c1 = p1; c2 = p2; return;
            }
            if (a <= ContactEps)
            {
                s = 0f; t = Math.Clamp(f / e, 0f, 1f);
            }
            else
            {
                float c = d1.Dot(r);
                if (e <= ContactEps)
                {
                    t = 0f; s = Math.Clamp(-c / a, 0f, 1f);
                }
                else
                {
                    float b = d1.Dot(d2); float denom = a * e - b * b;
                    s = denom != 0f ? Math.Clamp((b * f - c * e) / denom, 0f, 1f) : 0f;
                    t = (b * s + f) / e;
                    if (t < 0f) { t = 0f; s = Math.Clamp(-c / a, 0f, 1f); }
                    else if (t > 1f) { t = 1f; s = Math.Clamp((b - c) / a, 0f, 1f); }
                }
            }
            c1 = p1 + d1 * s;
            c2 = p2 + d2 * t;
        }

        // 与えた軸に垂直な単位ベクトル (縮退法線のフォールバック)。
        private static Vec3 PerpVector(Vec3 axis)
        {
            var a = axis.Normalized;
            var reference = Math.Abs(a.x) < 0.9f ? Vec3.XAxis : Vec3.YAxis;
            var perp = Vec3.Cross(a, reference);
            var len = perp.Length;
            return len > ContactEps ? perp / len : Vec3.YAxis;
        }

        /// <summary>
        /// A と B の貫入を GJK+EPA で解く (フォールバック用)。接触があれば true。
        /// </summary>
        private static bool GjkEpaPenetration(RigidBody a, RigidBody b, out ContactPoint contact)
        {
            contact = default;

            // --- GJK: 原点が Minkowski 差に含まれるか ---
            var simplex = new List<SupportVert>(4);
            var dir = a.WorldTransform.Origin - b.WorldTransform.Origin;
            if (dir.LengthSquared < Epsilon) dir = Vec3.XAxis;

            simplex.Add(Support(a, b, dir));
            dir = -simplex[0].V;

            for (int iter = 0; iter < MaxIterations; iter++)
            {
                if (dir.LengthSquared < Epsilon) break;
                var p = Support(a, b, dir);
                if (p.V.Dot(dir) < 0)
                    return false; // 原点を越えられない → 分離
                simplex.Add(p);
                if (DoSimplex(simplex, ref dir))
                {
                    // 原点包含 → EPA で貫入解決。
                    return Epa(a, b, simplex, out contact);
                }
            }
            return false;
        }

        // GJK simplex 更新。原点を含んだら true。
        private static bool DoSimplex(List<SupportVert> s, ref Vec3 dir)
        {
            if (s.Count == 2) return Line(s, ref dir);
            if (s.Count == 3) return Triangle(s, ref dir);
            return Tetrahedron(s, ref dir);
        }

        private static bool Line(List<SupportVert> s, ref Vec3 dir)
        {
            var a = s[1].V; var b = s[0].V;
            var ab = b - a; var ao = -a;
            if (ab.Dot(ao) > 0)
                dir = Vec3.Cross(Vec3.Cross(ab, ao), ab);
            else { s.RemoveAt(0); dir = ao; }
            return false;
        }

        private static bool Triangle(List<SupportVert> s, ref Vec3 dir)
        {
            var a = s[2].V; var b = s[1].V; var c = s[0].V;
            var ab = b - a; var ac = c - a; var ao = -a;
            var abc = Vec3.Cross(ab, ac);

            if (Vec3.Cross(abc, ac).Dot(ao) > 0)
            {
                if (ac.Dot(ao) > 0) { s.RemoveAt(1); dir = Vec3.Cross(Vec3.Cross(ac, ao), ac); }
                else return StarLine(s, ref dir);
            }
            else if (Vec3.Cross(ab, abc).Dot(ao) > 0)
            {
                return StarLine(s, ref dir);
            }
            else
            {
                if (abc.Dot(ao) > 0) { dir = abc; }
                else { (s[0], s[1]) = (s[1], s[0]); dir = -abc; }
                return false;
            }
            return false;
        }

        private static bool StarLine(List<SupportVert> s, ref Vec3 dir)
        {
            // 辺 AB を残す。
            s.RemoveAt(0);
            return Line(s, ref dir);
        }

        private static bool Tetrahedron(List<SupportVert> s, ref Vec3 dir)
        {
            var a = s[3].V; var b = s[2].V; var c = s[1].V; var d = s[0].V;
            var ao = -a;
            var abc = Vec3.Cross(b - a, c - a);
            var acd = Vec3.Cross(c - a, d - a);
            var adb = Vec3.Cross(d - a, b - a);

            if (abc.Dot(ao) > 0) { s.RemoveAt(0); dir = abc; return Triangle(s, ref dir); }
            if (acd.Dot(ao) > 0) { s.RemoveAt(2); dir = acd; return Triangle(s, ref dir); }
            if (adb.Dot(ao) > 0) { s.RemoveAt(1); dir = adb; return Triangle(s, ref dir); }
            return true; // 原点は四面体内部
        }

        // --- EPA: 貫入方向と深さ ---
        private struct Face { public int A, B, C; public Vec3 Normal; public float Dist; public bool Valid; }

        // 面数の上限。到達時はその時点の最良面で打ち切る (無限ループ/例外にしない)。
        private const int MaxFaces = 128;

        // 安全弁の発動回数 (診断用。PhysicsWorld.DebugContactCount と同様の public フィールド)。
        public static long EpaIterCapHits;   // 反復上限で打ち切った回数
        public static long EpaFaceCapHits;   // 面数上限で打ち切った回数

        private static bool Epa(RigidBody a, RigidBody b, List<SupportVert> simplex, out ContactPoint contact)
        {
            contact = default;
            if (simplex.Count < 4) { if (!ExpandToTetra(a, b, simplex)) return false; }

            var verts = new List<SupportVert>(simplex);
            // 初期四面体: 各面を対頂点から見て外向きになるよう巻き方向を正規化する。
            var faces = new List<Face>();
            AddIfValid(faces, MakeFaceOriented(verts, 0, 1, 2, 3));
            AddIfValid(faces, MakeFaceOriented(verts, 0, 1, 3, 2));
            AddIfValid(faces, MakeFaceOriented(verts, 0, 2, 3, 1));
            AddIfValid(faces, MakeFaceOriented(verts, 1, 2, 3, 0));

            Face closest = default;
            bool converged = false, faceCap = false;
            var edges = new List<(int, int)>();

            for (int iter = 0; iter < MaxIterations; iter++)
            {
                if (!TryFindClosestFace(faces, out closest)) break; // 有効面が無い → フォールバック

                var p = Support(a, b, closest.Normal);
                float d = p.V.Dot(closest.Normal);
                // 相対許容差での収束判定 (絶対値だと曲面で収束前に反復上限に達する)。
                if (d - closest.Dist < closest.Dist * 1e-3f + 1e-5f) { converged = true; break; }

                // p から見える面を削除し、輪郭 (horizon) を抽出。
                edges.Clear();
                for (int i = faces.Count - 1; i >= 0; i--)
                {
                    if (faces[i].Normal.Dot(p.V - verts[faces[i].A].V) > 0)
                    {
                        AddEdge(edges, faces[i].A, faces[i].B);
                        AddEdge(edges, faces[i].B, faces[i].C);
                        AddEdge(edges, faces[i].C, faces[i].A);
                        faces.RemoveAt(i);
                    }
                }
                int newIndex = verts.Count;
                verts.Add(p);
                // 新規面は巻き方向を輪郭から継承 (反転しない)。縮退面は破棄。
                foreach (var (e0, e1) in edges)
                    AddIfValid(faces, MakeFace(verts, e0, e1, newIndex));

                if (faces.Count == 0) break;
                if (faces.Count > MaxFaces) { faceCap = true; break; }
            }

            if (faceCap) EpaFaceCapHits++;
            else if (!converged) EpaIterCapHits++;

            // 打ち切り後も最良の有効面で接触を返す。
            if (!TryFindClosestFace(faces, out closest))
            {
                // 有効面ゼロの退化: NaN を返さず中心方向の安全な法線でフォールバック。
                var dir = b.WorldTransform.Origin - a.WorldTransform.Origin;
                var nf = dir.LengthSquared > Epsilon ? dir.Normalized : Vec3.YAxis;
                var pa0 = a.WorldTransform.Origin;
                var pb0 = b.WorldTransform.Origin;
                FillContact(a, b, ref contact, nf, -Epsilon, pa0, pb0);
                return true;
            }

            BarycentricProject(verts, closest, out var baryA, out var baryB);
            FillContact(a, b, ref contact, closest.Normal, -closest.Dist, baryA, baryB);
            return true;
        }

        private static void FillContact(RigidBody a, RigidBody b, ref ContactPoint contact,
            Vec3 normal, float distance, Vec3 baryA, Vec3 baryB)
        {
            contact.Normal = normal;
            contact.Distance = distance; // 貫入は負
            contact.PositionWorldA = baryA;
            contact.PositionWorldB = baryB;
            // 剛体移動後に再投影できるようローカル座標も保持。
            contact.LocalPointA = a.WorldTransform.InverseTransformPoint(baryA);
            contact.LocalPointB = b.WorldTransform.InverseTransformPoint(baryB);
        }

        private static void AddIfValid(List<Face> faces, Face f)
        {
            if (f.Valid) faces.Add(f);
        }

        private static bool ExpandToTetra(RigidBody a, RigidBody b, List<SupportVert> s)
        {
            // 退化 simplex を四面体まで補う (稀ケースの保険)。
            var dirs = new[] { Vec3.XAxis, Vec3.YAxis, Vec3.ZAxis, -Vec3.XAxis, -Vec3.YAxis, -Vec3.ZAxis };
            foreach (var d in dirs)
            {
                if (s.Count >= 4) break;
                var p = Support(a, b, d);
                bool dup = false;
                foreach (var e in s) if ((e.V - p.V).LengthSquared < 1e-8f) { dup = true; break; }
                if (!dup) s.Add(p);
            }
            return s.Count >= 4;
        }

        // 巻き方向から外向き法線を一意に決める (反転しない)。縮退面は Valid=false。
        private static Face MakeFace(List<SupportVert> v, int a, int b, int c)
        {
            var n = Vec3.Cross(v[b].V - v[a].V, v[c].V - v[a].V);
            var len = n.Length;
            if (len <= Epsilon) return new Face { Valid = false }; // 縮退は破棄
            n /= len;
            var dist = n.Dot(v[a].V);
            return new Face { A = a, B = b, C = c, Normal = n, Dist = dist, Valid = true };
        }

        // 初期四面体用: 対頂点 opp から見て外向きになるよう頂点順序を正規化する。
        private static Face MakeFaceOriented(List<SupportVert> v, int a, int b, int c, int opp)
        {
            var n = Vec3.Cross(v[b].V - v[a].V, v[c].V - v[a].V);
            var len = n.Length;
            if (len <= Epsilon) return new Face { Valid = false };
            n /= len;
            // n が対頂点側を向いていたら内向き → 巻き方向を反転して外向きに揃える。
            if (n.Dot(v[opp].V - v[a].V) > 0) { (b, c) = (c, b); n = -n; }
            var dist = n.Dot(v[a].V);
            return new Face { A = a, B = b, C = c, Normal = n, Dist = dist, Valid = true };
        }

        // 原点に最も近い有効面を返す。原点が外側になる不正面 (Dist<0) は無視。
        private static bool TryFindClosestFace(List<Face> faces, out Face best)
        {
            best = default; bool found = false; float bd = float.MaxValue;
            for (int i = 0; i < faces.Count; i++)
            {
                var f = faces[i];
                if (!f.Valid || f.Dist < -1e-6f) continue;
                if (f.Dist < bd) { bd = f.Dist; best = f; found = true; }
            }
            return found;
        }

        private static void AddEdge(List<(int, int)> edges, int a, int b)
        {
            // 反対向きの辺があれば相殺 (穴の輪郭抽出)。
            for (int i = 0; i < edges.Count; i++)
                if (edges[i].Item1 == b && edges[i].Item2 == a) { edges.RemoveAt(i); return; }
            edges.Add((a, b));
        }

        private static void BarycentricProject(List<SupportVert> v, Face f, out Vec3 onA, out Vec3 onB)
        {
            // 原点を面へ射影し、重心座標で A/B 上の witness を補間。
            var pa = v[f.A]; var pb = v[f.B]; var pc = v[f.C];
            var proj = f.Normal * f.Dist;
            Barycentric(proj, pa.V, pb.V, pc.V, out var u, out var vv, out var w);
            onA = pa.A * u + pb.A * vv + pc.A * w;
            onB = pa.B * u + pb.B * vv + pc.B * w;
        }

        private static void Barycentric(Vec3 p, Vec3 a, Vec3 b, Vec3 c,
            out float u, out float v, out float w)
        {
            var v0 = b - a; var v1 = c - a; var v2 = p - a;
            var d00 = v0.Dot(v0); var d01 = v0.Dot(v1); var d11 = v1.Dot(v1);
            var d20 = v2.Dot(v0); var d21 = v2.Dot(v1);
            var denom = d00 * d11 - d01 * d01;
            if (Math.Abs(denom) < Epsilon) { u = 1; v = 0; w = 0; return; }
            v = (d11 * d20 - d01 * d21) / denom;
            w = (d00 * d21 - d01 * d20) / denom;
            u = 1f - v - w;
        }
    }
}
