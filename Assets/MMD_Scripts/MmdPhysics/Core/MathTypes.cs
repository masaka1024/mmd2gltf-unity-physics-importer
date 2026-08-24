// ===========================================================================
// Unity Bullet 互換物理エンジン – Core Math
// Bullet 2.75 の実装に合わせた高精度 3D 数学型
// ===========================================================================

using System;

namespace BulletPhysics
{
    /// <summary>
    /// Bullet と同じ順序 (x,y,z) の 3D ベクトル。
    /// PmxEditor の float3 / Vector3 と同等。
    /// </summary>
    public struct Vec3 : IEquatable<Vec3>
    {
        public float x, y, z;

        public float this[int index]
        {
            get
            {
                return index switch
                {
                    0 => x,
                    1 => y,
                    2 => z,
                    _ => throw new ArgumentOutOfRangeException(nameof(index))
                };
            }
            set
            {
                switch (index)
                {
                    case 0: x = value; break;
                    case 1: y = value; break;
                    case 2: z = value; break;
                    default: throw new ArgumentOutOfRangeException(nameof(index));
                }
            }
        }

        public static readonly Vec3 Zero = new(0, 0, 0);
        public static readonly Vec3 One = new(1, 1, 1);
        public static readonly Vec3 XAxis = new(1, 0, 0);
        public static readonly Vec3 YAxis = new(0, 1, 0);
        public static readonly Vec3 ZAxis = new(0, 0, 1);

        public float LengthSquared => x * x + y * y + z * z;
        public float Length => (float)Math.Sqrt(LengthSquared);

        public Vec3 Normalized
        {
            get
            {
                var len = Length;
                return len > 0 ? this / len : Zero;
            }
        }

        // 注: パラメータなしコンストラクタは C# 10 以降の機能 (Unity=C# 9 では不可) のため置かない。
        //     既定 new Vec3() は全フィールド 0 となり、旧コンストラクタ (全 0) と同一の意味。
        public Vec3(float v) { x = v; y = v; z = v; }
        public Vec3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }

        // Operators
        public static Vec3 operator +(Vec3 a, Vec3 b) => new(a.x + b.x, a.y + b.y, a.z + b.z);
        public static Vec3 operator -(Vec3 a, Vec3 b) => new(a.x - b.x, a.y - b.y, a.z - b.z);
        public static Vec3 operator -(Vec3 a) => new(-a.x, -a.y, -a.z);
        public static Vec3 operator *(Vec3 a, float s) => new(a.x * s, a.y * s, a.z * s);
        public static Vec3 operator *(float s, Vec3 a) => a * s;
        public static Vec3 operator /(Vec3 a, float s) { var i = 1f / s; return a * i; }
        public static bool operator ==(Vec3 a, Vec3 b) => a.Equals(b);
        public static bool operator !=(Vec3 a, Vec3 b) => !a.Equals(b);
        public static explicit operator UnityEngine.Vector3(Vec3 v) => new(v.x, v.y, v.z);
        public static explicit operator Vec3(UnityEngine.Vector3 v) => new(v.x, v.y, v.z);

        // Bullet-style functions
        public static Vec3 Cross(Vec3 a, Vec3 b)
        {
            return new Vec3(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            );
        }

        public float Dot(Vec3 b) => x * b.x + y * b.y + z * b.z;

        public float Angle(Vec3 b)
        {
            var d = Dot(b) / (Length * b.Length);
            return (float)Math.Acos(Math.Clamp(d, -1f, 1f));
        }

        public bool Equals(Vec3 other)
        {
            return Math.Abs(x - other.x) < 1e-6f &&
                   Math.Abs(y - other.y) < 1e-6f &&
                   Math.Abs(z - other.z) < 1e-6f;
        }

        public override bool Equals(object obj) => obj is Vec3 v && Equals(v);
        public override int GetHashCode() => HashCode.Combine(x, y, z);
        public override string ToString() => $"({x:F4}, {y:F4}, {z:F4})";
    }

    /// <summary>
    /// Bullet と同じ Quaternion (x, y, z, w) – w は実部。
    /// MMD のボーン回転クォータニオンと互換。
    /// </summary>
    public struct Quat : IEquatable<Quat>
    {
        public float x, y, z, w;

        public static readonly Quat Identity = new(0, 0, 0, 1);

        // 注: パラメータなしコンストラクタは C# 10 以降の機能 (Unity=C# 9 では不可) のため置かない。
        //     旧コンストラクタは w=1 (単位クォータニオン) を既定にしていたが、C# 9 の既定 new Quat() は
        //     全 0 (w=0) となり意味が変わる。単位が必要な箇所は必ず Quat.Identity を使うこと。
        //     (現状 new Quat() の呼び出しは無く、配列 new Quat[] は元々ゼロ初期化で不変)。
        public Quat(float x, float y, float z, float w) { this.x = x; this.y = y; this.z = z; this.w = w; }

        // Convert from axis-angle (Bullet の回転表現と同等)
        public static Quat FromAxisAngle(Vec3 axis, float angleRad)
        {
            var half = angleRad * 0.5f;
            var sin = (float)Math.Sin(half);
            var n = axis.Normalized;
            return new Quat(n.x * sin, n.y * sin, n.z * sin, (float)Math.Cos(half));
        }

        /// <summary>
        /// MMD/PMX のオイラー角(ラジアン)→クォータニオン。適用順は Z→X→Y、
        /// すなわち R = Ry * Rx * Rz (YXZ順)。PMX の剛体/Joint の回転はこの順序で定義されている
        /// (Bullet の btQuaternion(yaw, pitch, roll) と同型。saba 等の実装も同じ)。
        ///
        /// ★2026-08-10 修正: 従来は FromEuler(=ZYX順) を使っており、複合回転の剛体で
        ///   向きがズレていた。「髪のカプセルが髪の流れに沿わない」の原因。
        ///   全13モデルで実測: カプセルのY軸とボーン→子ボーン方向のズレ角(中央値)は
        ///   YXZ 0.0°/ZYX 1.6° (モデルA)、YXZ 0.0°/ZYX 5.8° (モデルC)。
        ///   15°以内に収まる割合も YXZ 91.8% / ZYX 80.0% (同モデル) と YXZ が優る。
        ///   撤去した旧PhysXインポーターが Quaternion.Euler(=Unityの YXZ) を使っていたのとも一致する。
        /// </summary>
        public static Quat FromEulerYxz(float rx, float ry, float rz)
        {
            var cx = (float)Math.Cos(rx * 0.5f); var sx = (float)Math.Sin(rx * 0.5f);
            var cy = (float)Math.Cos(ry * 0.5f); var sy = (float)Math.Sin(ry * 0.5f);
            var cz = (float)Math.Cos(rz * 0.5f); var sz = (float)Math.Sin(rz * 0.5f);

            // q = qy * qx * qz を展開したもの。
            return new Quat(
                cy * sx * cz + sy * cx * sz, // x
                sy * cx * cz - cy * sx * sz, // y
                cy * cx * sz - sy * sx * cz, // z
                cy * cx * cz + sy * sx * sz  // w
            );
        }

        // Convert from Euler angles (radians) – ZYX order (Roll-Pitch-Yaw)
        // ※PMX の回転はこの順序では**ない**。PMX データには FromEulerYxz を使うこと。
        public static Quat FromEuler(float rx, float ry, float rz)
        {
            var cx = (float)Math.Cos(rx * 0.5f); var sx = (float)Math.Sin(rx * 0.5f);
            var cy = (float)Math.Cos(ry * 0.5f); var sy = (float)Math.Sin(ry * 0.5f);
            var cz = (float)Math.Cos(rz * 0.5f); var sz = (float)Math.Sin(rz * 0.5f);

            return new Quat(
                sx * cy * cz - cx * sy * sz, // x
                cx * sy * cz + sx * cy * sz, // y
                cx * cy * sz - sx * sy * cz, // z
                cx * cy * cz + sx * sy * sz  // w
            );
        }

        public static Quat operator *(Quat a, Quat b)
        {
            return new Quat(
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
            );
        }

        public static Vec3 operator *(Quat q, Vec3 v)
        {
            // q * v * q^(-1) via conjugate for unit quaternion
            float qx = q.x, qy = q.y, qz = q.z, qw = q.w;
            float ix = v.x, iy = v.y, iz = v.z;
            var t = 2f / (qx * qx + qy * qy + qz * qz + qw * qw);

            var ox = (1f - t * (qy * qy + qz * qz)) * ix
                   + t * (qx * qy - qw * qz) * iy
                   + t * (qx * qz + qw * qy) * iz;
            var oy = t * (qx * qy + qw * qz) * ix
                   + (1f - t * (qx * qx + qz * qz)) * iy
                   + t * (qy * qz - qw * qx) * iz;
            var oz = t * (qx * qz - qw * qy) * ix
                   + t * (qy * qz + qw * qx) * iy
                   + (1f - t * (qx * qx + qy * qy)) * iz;

            return new Vec3(ox, oy, oz);
        }

        public static Quat FromMatrix(Matrix4x4 m)
        {
            var trace = m.m00 + m.m11 + m.m22;
            Quat q;

            if (trace > 0)
            {
                var s = (float)Math.Sqrt(trace + 1f) * 2f;
                q = new Quat(
                    (m.m21 - m.m12) / s,
                    (m.m02 - m.m20) / s,
                    (m.m10 - m.m01) / s,
                    0.25f * s);
            }
            else
            {
                if (m.m00 > m.m11 && m.m00 > m.m22)
                {
                    var s = (float)Math.Sqrt(1f + m.m00 - m.m11 - m.m22) * 2f;
                    q = new Quat(
                        0.25f * s,
                        (m.m01 + m.m10) / s,
                        (m.m02 + m.m20) / s,
                        (m.m21 - m.m12) / s);
                }
                else if (m.m11 > m.m22)
                {
                    var s = (float)Math.Sqrt(1f + m.m11 - m.m00 - m.m22) * 2f;
                    q = new Quat(
                        (m.m01 + m.m10) / s,
                        0.25f * s,
                        (m.m12 + m.m21) / s,
                        (m.m02 - m.m20) / s);
                }
                else
                {
                    var s = (float)Math.Sqrt(1f + m.m22 - m.m00 - m.m11) * 2f;
                    q = new Quat(
                        (m.m02 + m.m20) / s,
                        (m.m12 + m.m21) / s,
                        0.25f * s,
                        (m.m10 - m.m01) / s);
                }
            }
            return q;
        }

        public Vec3 Rotate(Vec3 v) => this * v;

        public Quat Normalized
        {
            get
            {
                var len = (float)Math.Sqrt(x * x + y * y + z * z + w * w);
                return len > 0 ? this / len : Identity;
            }
        }

        public static Quat operator /(Quat q, float s)
        {
            var i = 1f / s;
            return new Quat(q.x * i, q.y * i, q.z * i, q.w * i);
        }

        public static Quat operator *(Quat q, float s)
        {
            return new Quat(q.x * s, q.y * s, q.z * s, q.w * s);
        }

        public static Quat Slerp(Quat a, Quat b, float t)
        {
            var cos = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
            var wrap = false;
            if (cos < 0) { cos = -cos; wrap = true; }

            float s0, s1;
            if ((1f - cos) > 0.001f)
            {
                var omega = (float)Math.Acos(cos);
                var sin = (float)Math.Sin(omega);
                s0 = (float)Math.Sin((1f - t) * omega) / sin;
                s1 = (float)Math.Sin(t * omega) / sin;
            }
            else
            {
                s0 = 1f - t;
                s1 = t;
            }

            if (wrap) { s1 = -s1; }
            return new Quat(
                s0 * a.x + s1 * b.x,
                s0 * a.y + s1 * b.y,
                s0 * a.z + s1 * b.z,
                s0 * a.w + s1 * b.w);
        }

        public Quat Conjugated() => new(-x, -y, -z, w);

        public override string ToString() => $"q({x:F4}, {y:F4}, {z:F4}, {w:F4})";

        public bool Equals(Quat other)
        {
            return Math.Abs(x - other.x) < 1e-6f &&
                   Math.Abs(y - other.y) < 1e-6f &&
                   Math.Abs(z - other.z) < 1e-6f &&
                   Math.Abs(w - other.w) < 1e-6f;
        }
        public override bool Equals(object obj) => obj is Quat q && Equals(q);
        public override int GetHashCode() => HashCode.Combine(x, y, z, w);
    }

    /// <summary>
    /// 4x4 同次変換行列 (Row-major)。
    /// MMD ボーンのローカル行列 = Scale * Rotation * Translation
    /// </summary>
    public struct Matrix4x4 : IEquatable<Matrix4x4>
    {
        public float m00, m01, m02, m03;
        public float m10, m11, m12, m13;
        public float m20, m21, m22, m23;
        public float m30, m31, m32, m33;

        // 注: パラメータなしコンストラクタは C# 10 以降の機能 (Unity=C# 9 では不可) のため置かない。
        //     旧コンストラクタは単位行列を既定にしていたが、C# 9 の既定 new Matrix4x4() は全 0 になる。
        //     単位行列は下の Identity を使うこと (対角のみ 1、オブジェクト初期化子で明示構築)。
        public static Matrix4x4 Identity => new Matrix4x4
        {
            m00 = 1, m01 = 0, m02 = 0, m03 = 0,
            m10 = 0, m11 = 1, m12 = 0, m13 = 0,
            m20 = 0, m21 = 0, m22 = 1, m23 = 0,
            m30 = 0, m31 = 0, m32 = 0, m33 = 1,
        };

        public static Matrix4x4 Translation(Vec3 t)
        {
            var m = Identity;
            m.m03 = t.x; m.m13 = t.y; m.m23 = t.z;
            return m;
        }

        public static Matrix4x4 Rotation(Quat q)
        {
            float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
            float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
            float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

            return new Matrix4x4
            {
                m00 = 1f - 2f * (yy + zz), m01 = 2f * (xy + wz),     m02 = 2f * (xz - wy),     m03 = 0,
                m10 = 2f * (xy - wz),      m11 = 1f - 2f * (xx + zz), m12 = 2f * (yz + wx),     m13 = 0,
                m20 = 2f * (xz + wy),      m21 = 2f * (yz - wx),      m22 = 1f - 2f * (xx + yy), m23 = 0,
                m30 = 0,                   m31 = 0,                   m32 = 0,                   m33 = 1,
            };
        }

        public static Matrix4x4 Scale(Vec3 s)
        {
            return new Matrix4x4
            {
                m00 = s.x, m01 = 0,     m02 = 0,     m03 = 0,
                m10 = 0,     m11 = s.y, m12 = 0,     m13 = 0,
                m20 = 0,     m21 = 0,     m22 = s.z, m23 = 0,
                m30 = 0,     m31 = 0,     m32 = 0,     m33 = 1,
            };
        }

        public static Matrix4x4 Multiply(Matrix4x4 a, Matrix4x4 b)
        {
            return new Matrix4x4
            {
                m00 = a.m00 * b.m00 + a.m01 * b.m10 + a.m02 * b.m20 + a.m03 * b.m30,
                m01 = a.m00 * b.m01 + a.m01 * b.m11 + a.m02 * b.m21 + a.m03 * b.m31,
                m02 = a.m00 * b.m02 + a.m01 * b.m12 + a.m02 * b.m22 + a.m03 * b.m32,
                m03 = a.m00 * b.m03 + a.m01 * b.m13 + a.m02 * b.m23 + a.m03 * b.m33,

                m10 = a.m10 * b.m00 + a.m11 * b.m10 + a.m12 * b.m20 + a.m13 * b.m30,
                m11 = a.m10 * b.m01 + a.m11 * b.m11 + a.m12 * b.m21 + a.m13 * b.m31,
                m12 = a.m10 * b.m02 + a.m11 * b.m12 + a.m12 * b.m22 + a.m13 * b.m32,
                m13 = a.m10 * b.m03 + a.m11 * b.m13 + a.m12 * b.m23 + a.m13 * b.m33,

                m20 = a.m20 * b.m00 + a.m21 * b.m10 + a.m22 * b.m20 + a.m23 * b.m30,
                m21 = a.m20 * b.m01 + a.m21 * b.m11 + a.m22 * b.m21 + a.m23 * b.m31,
                m22 = a.m20 * b.m02 + a.m21 * b.m12 + a.m22 * b.m22 + a.m23 * b.m32,
                m23 = a.m20 * b.m03 + a.m21 * b.m13 + a.m22 * b.m23 + a.m23 * b.m33,

                m30 = a.m30 * b.m00 + a.m31 * b.m10 + a.m32 * b.m20 + a.m33 * b.m30,
                m31 = a.m30 * b.m01 + a.m31 * b.m11 + a.m32 * b.m21 + a.m33 * b.m31,
                m32 = a.m30 * b.m02 + a.m31 * b.m12 + a.m32 * b.m22 + a.m33 * b.m32,
                m33 = a.m30 * b.m03 + a.m31 * b.m13 + a.m32 * b.m23 + a.m33 * b.m33,
            };
        }

        public Vec3 TransformPoint(Vec3 p)
        {
            var w = m03 * p.x + m13 * p.y + m23 * p.z + m33;
            return new Vec3(
                (m00 * p.x + m10 * p.y + m20 * p.z + m30) / w,
                (m01 * p.x + m11 * p.y + m21 * p.z + m31) / w,
                (m02 * p.x + m12 * p.y + m22 * p.z + m32) / w
            );
        }

        public Vec3 TransformVector(Vec3 v)
        {
            return new Vec3(
                m00 * v.x + m10 * v.y + m20 * v.z,
                m01 * v.x + m11 * v.y + m21 * v.z,
                m02 * v.x + m12 * v.y + m22 * v.z
            );
        }

        public static explicit operator UnityEngine.Matrix4x4(Matrix4x4 m)
        {
            var u = new UnityEngine.Matrix4x4();
            u.m00 = m.m00; u.m01 = m.m01; u.m02 = m.m02; u.m03 = m.m03;
            u.m10 = m.m10; u.m11 = m.m11; u.m12 = m.m12; u.m13 = m.m13;
            u.m20 = m.m20; u.m21 = m.m21; u.m22 = m.m22; u.m23 = m.m23;
            u.m30 = m.m30; u.m31 = m.m31; u.m32 = m.m32; u.m33 = m.m33;
            return u;
        }

        public static explicit operator Matrix4x4(UnityEngine.Matrix4x4 u)
        {
            return new Matrix4x4
            {
                m00 = u.m00, m01 = u.m01, m02 = u.m02, m03 = u.m03,
                m10 = u.m10, m11 = u.m11, m12 = u.m12, m13 = u.m13,
                m20 = u.m20, m21 = u.m21, m22 = u.m22, m23 = u.m23,
                m30 = u.m30, m31 = u.m31, m32 = u.m32, m33 = u.m33,
            };
        }

        public bool Equals(Matrix4x4 other)
        {
            var e = 1e-6f;
            return Math.Abs(m00 - other.m00) < e && Math.Abs(m01 - other.m01) < e &&
                   Math.Abs(m02 - other.m02) < e && Math.Abs(m03 - other.m03) < e &&
                   Math.Abs(m10 - other.m10) < e && Math.Abs(m11 - other.m11) < e &&
                   Math.Abs(m12 - other.m12) < e && Math.Abs(m13 - other.m13) < e &&
                   Math.Abs(m20 - other.m20) < e && Math.Abs(m21 - other.m21) < e &&
                   Math.Abs(m22 - other.m22) < e && Math.Abs(m23 - other.m23) < e &&
                   Math.Abs(m30 - other.m30) < e && Math.Abs(m31 - other.m31) < e &&
                   Math.Abs(m32 - other.m32) < e && Math.Abs(m33 - other.m33) < e;
        }
        public override bool Equals(object obj) => obj is Matrix4x4 m && Equals(m);
        public override int GetHashCode() => HashCode.Combine(
            HashCode.Combine(m00, m01, m02, m03, m10, m11, m12, m13),
            HashCode.Combine(m20, m21, m22, m23, m30, m31, m32, m33));
        public override string ToString()
        {
            return $"[{m00:F2},{m01:F2},{m02:F2},{m03:F2}]\n" +
                   $"[{m10:F2},{m11:F2},{m12:F2},{m13:F2}]\n" +
                   $"[{m20:F2},{m21:F2},{m22:F2},{m23:F2}]\n" +
                   $"[{m30:F2},{m31:F2},{m32:F2},{m33:F2}]";
        }
    }

    /// <summary>
    /// 無限平面 (地面) _collision_ のための法線-距離構造体。
    /// </summary>
    public struct Plane
    {
        public Vec3 Normal;
        public float Distance; // Plane: n·x + d = 0 の d

        public Plane(Vec3 normal, float distance)
        {
            Normal = normal.Normalized;
            Distance = distance;
        }

        // Signed distance from point to plane
        public float SignedDistance(Vec3 p) => Normal.Dot(p) + Distance;
    }

    /// <summary>
    /// Axis-Aligned Bounding Box.
    /// BroadPhase 用。
    /// </summary>
    public struct Aabb
    {
        public Vec3 Min, Max;

        public Aabb(Vec3 min, Vec3 max) { Min = min; Max = max; }

        public bool Contains(Vec3 p) => p.x >= Min.x && p.x <= Max.x &&
                                         p.y >= Min.y && p.y <= Max.y &&
                                         p.z >= Min.z && p.z <= Max.z;

        public bool Intersects(ref Aabb other)
        {
            return Min.x <= other.Max.x && Max.x >= other.Min.x &&
                   Min.y <= other.Max.y && Max.y >= other.Min.y &&
                   Min.z <= other.Max.z && Max.z >= other.Min.z;
        }

        public Vec3 Center => (Min + Max) * 0.5f;
        public Vec3 Extents => (Max - Min) * 0.5f;

        public void Expand(Vec3 p)
        {
            Min.x = Math.Min(Min.x, p.x); Min.y = Math.Min(Min.y, p.y); Min.z = Math.Min(Min.z, p.z);
            Max.x = Math.Max(Max.x, p.x); Max.y = Math.Max(Max.y, p.y); Max.z = Math.Max(Max.z, p.z);
        }

        public void Expand(float radius)
        {
            Min.x -= radius; Min.y -= radius; Min.z -= radius;
            Max.x += radius; Max.y += radius; Max.z += radius;
        }
    }
}
