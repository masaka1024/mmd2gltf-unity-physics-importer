// ===========================================================================
// Unity Bullet 互換物理エンジン – Collision Shapes
// PMX 剛体形状: 0:球 1:箱 2:カプセル
// Bullet の btSphereShape / btBoxShape / btCapsuleShape に対応。
// ===========================================================================

using System;

namespace BulletPhysics
{
    public enum ShapeType
    {
        Sphere = 0,   // PMX 形状 0
        Box = 1,      // PMX 形状 1
        Capsule = 2,  // PMX 形状 2
    }

    /// <summary>
    /// 凸形状の抽象基底。GJK/EPA 用の support 関数と慣性計算を提供。
    /// </summary>
    public abstract class CollisionShape
    {
        public abstract ShapeType Type { get; }

        /// <summary>衝突マージン。Bullet 既定に合わせ既定値を持つ。</summary>
        public float Margin = 0.04f;

        /// <summary>ローカル座標系での support point (方向 dir で最も遠い点)。</summary>
        public abstract Vec3 LocalSupport(Vec3 dir);

        /// <summary>マージンを含む support point。</summary>
        public Vec3 LocalSupportWithMargin(Vec3 dir)
        {
            var d = dir;
            var len2 = d.LengthSquared;
            if (len2 < 1e-12f) return LocalSupport(Vec3.XAxis);
            return LocalSupport(d) + d.Normalized * Margin;
        }

        /// <summary>質量 mass に対するローカル慣性テンソル (対角成分)。</summary>
        public abstract Vec3 CalculateLocalInertia(float mass);

        /// <summary>ローカル AABB 半径 (原点中心の外接半径)。</summary>
        public abstract float BoundingRadius { get; }
    }

    /// <summary>球。PMX size.x を半径として使用。</summary>
    public sealed class SphereShape : CollisionShape
    {
        public float Radius;

        public SphereShape(float radius) { Radius = radius; Margin = radius; }

        public override ShapeType Type => ShapeType.Sphere;

        // 球の support は常に中心 (半径はマージン扱い)。
        public override Vec3 LocalSupport(Vec3 dir) => Vec3.Zero;

        public override Vec3 CalculateLocalInertia(float mass)
        {
            var i = 0.4f * mass * Radius * Radius; // 2/5 m r^2
            return new Vec3(i, i, i);
        }

        public override float BoundingRadius => Radius;
    }

    /// <summary>箱。PMX size(x,y,z) は半径 (half-extents)。</summary>
    public sealed class BoxShape : CollisionShape
    {
        public Vec3 HalfExtents;

        public BoxShape(Vec3 halfExtents)
        {
            HalfExtents = halfExtents;
            Margin = Math.Min(Math.Min(halfExtents.x, halfExtents.y), halfExtents.z) * 0.04f;
        }

        public override ShapeType Type => ShapeType.Box;

        public override Vec3 LocalSupport(Vec3 dir)
        {
            // マージン分を差し引いたコア半径。
            var h = HalfExtents - new Vec3(Margin);
            return new Vec3(
                dir.x >= 0 ? h.x : -h.x,
                dir.y >= 0 ? h.y : -h.y,
                dir.z >= 0 ? h.z : -h.z);
        }

        public override Vec3 CalculateLocalInertia(float mass)
        {
            var x = HalfExtents.x * 2f;
            var y = HalfExtents.y * 2f;
            var z = HalfExtents.z * 2f;
            var c = mass / 12f;
            return new Vec3(
                c * (y * y + z * z),
                c * (x * x + z * z),
                c * (x * x + y * y));
        }

        public override float BoundingRadius => HalfExtents.Length;
    }

    /// <summary>
    /// カプセル。PMX: size.x = 半径, size.y = 高さ(中心の直線長)。Y 軸方向。
    /// Bullet btCapsuleShape と同様、線分 + 半径。
    /// </summary>
    public sealed class CapsuleShape : CollisionShape
    {
        public float Radius;
        public float Height; // 円柱部分の長さ (両端の半球は含まない)

        public CapsuleShape(float radius, float height)
        {
            Radius = radius;
            Height = height;
            Margin = radius;
        }

        public override ShapeType Type => ShapeType.Capsule;

        public float HalfHeight => Height * 0.5f;

        public override Vec3 LocalSupport(Vec3 dir)
        {
            // 線分の端点 (半径はマージンで付与)。
            return dir.y >= 0
                ? new Vec3(0, HalfHeight, 0)
                : new Vec3(0, -HalfHeight, 0);
        }

        public override Vec3 CalculateLocalInertia(float mass)
        {
            // Bullet btCapsuleShape::calculateLocalInertia を忠実に再現する。
            // Bullet は「両端の球を含む外接箱」の慣性で近似している
            // (原文コメント: "as an approximation, take the inertia of the box that
            //  bounds the spheres")。物理的に正しい円柱+半球の解析式ではないが、
            // 本家(Bullet 2.75/PMX)との挙動互換を優先し、あえてこの近似を使う。
            // ※ 将来「正しい式」に直さないこと。円柱式は横軸慣性が半分以下になり、
            //   カプセル剛体(髪など)が本家より過敏に回ってしまう。
            // upAxis は Y (PMX のカプセルは Y 軸方向)。halfExtents = (r,r,r) の Y に halfHeight を加算。
            var r = Radius;
            var hx = r;
            var hy = r + HalfHeight;
            var hz = r;
            var lx = 2f * hx;
            var ly = 2f * hy;
            var lz = 2f * hz;
            var c = mass / 12f; // Bullet の scaledmass = mass * 0.08333333
            return new Vec3(
                c * (ly * ly + lz * lz),  // X
                c * (lx * lx + lz * lz),  // Y (up)
                c * (lx * lx + ly * ly)); // Z
        }

        public override float BoundingRadius => HalfHeight + Radius;
    }
}
