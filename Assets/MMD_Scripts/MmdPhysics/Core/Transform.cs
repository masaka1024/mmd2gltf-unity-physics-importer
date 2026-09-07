// ===========================================================================
// Unity Bullet 互換物理エンジン – RigidTransform
// Bullet の btTransform (basis + origin) と等価な剛体変換。
// ===========================================================================

using System;

namespace BulletPhysics
{
    /// <summary>
    /// 剛体の姿勢を表す変換 (回転 + 平行移動)。Bullet の btTransform に対応。
    /// スケールを含まないため逆変換を安価に計算できる。
    /// </summary>
    public struct RigidTransform : IEquatable<RigidTransform>
    {
        public Quat Rotation;   // basis
        public Vec3 Origin;     // translation

        public static readonly RigidTransform Identity = new(Quat.Identity, Vec3.Zero);

        public RigidTransform(Quat rotation, Vec3 origin)
        {
            Rotation = rotation;
            Origin = origin;
        }

        /// <summary>ローカル点をワールド点へ変換 (basis * p + origin)。</summary>
        public Vec3 TransformPoint(Vec3 p) => Rotation * p + Origin;

        /// <summary>方向ベクトルを回転のみ適用して変換 (平行移動なし)。</summary>
        public Vec3 TransformDirection(Vec3 v) => Rotation * v;

        /// <summary>ワールド点をローカル点へ変換 (逆変換)。</summary>
        public Vec3 InverseTransformPoint(Vec3 p) => Rotation.Conjugated() * (p - Origin);

        /// <summary>ワールド方向をローカル方向へ変換。</summary>
        public Vec3 InverseTransformDirection(Vec3 v) => Rotation.Conjugated() * v;

        /// <summary>逆変換を返す。</summary>
        public RigidTransform Inverse()
        {
            var invRot = Rotation.Conjugated();
            return new RigidTransform(invRot, invRot * (-Origin));
        }

        /// <summary>this * rhs を合成 (rhs を先に、this を後に適用)。</summary>
        public static RigidTransform operator *(RigidTransform a, RigidTransform b)
        {
            return new RigidTransform(
                (a.Rotation * b.Rotation).Normalized,
                a.Rotation * b.Origin + a.Origin);
        }

        /// <summary>this から other への相対変換 (this^-1 * other)。</summary>
        public RigidTransform InverseTimes(RigidTransform other)
        {
            var v = other.Origin - Origin;
            var invRot = Rotation.Conjugated();
            return new RigidTransform(invRot * other.Rotation, invRot * v);
        }

        public Matrix4x4 ToMatrix()
        {
            var m = Matrix4x4.Rotation(Rotation);
            m.m03 = Origin.x; m.m13 = Origin.y; m.m23 = Origin.z;
            return m;
        }

        /// <summary>PMX の (位置, オイラー角ラジアン) から剛体変換を作る。
        /// 回転は MMD/PMX の YXZ 順で解釈する (Quat.FromEulerYxz のコメント参照)。</summary>
        public static RigidTransform FromEuler(Vec3 posRad, Vec3 eulerRad)
        {
            return new RigidTransform(Quat.FromEulerYxz(eulerRad.x, eulerRad.y, eulerRad.z), posRad);
        }

        public bool Equals(RigidTransform other) =>
            Rotation.Equals(other.Rotation) && Origin.Equals(other.Origin);

        public override bool Equals(object obj) => obj is RigidTransform t && Equals(t);
        public override int GetHashCode() => HashCode.Combine(Rotation, Origin);
        public override string ToString() => $"T[{Origin}, {Rotation}]";
    }

    /// <summary>
    /// 3x3 行列。慣性テンソル / 姿勢 basis 計算用。Row-major。
    /// </summary>
    public struct Matrix3x3
    {
        public Vec3 Row0, Row1, Row2;

        public Matrix3x3(Vec3 r0, Vec3 r1, Vec3 r2) { Row0 = r0; Row1 = r1; Row2 = r2; }

        public static readonly Matrix3x3 Identity =
            new(Vec3.XAxis, Vec3.YAxis, Vec3.ZAxis);

        public static readonly Matrix3x3 Zero =
            new(Vec3.Zero, Vec3.Zero, Vec3.Zero);

        public static Matrix3x3 Diagonal(Vec3 d) =>
            new(new Vec3(d.x, 0, 0), new Vec3(0, d.y, 0), new Vec3(0, 0, d.z));

        /// <summary>★2026-09-07: 64バイトの Matrix4x4 を組んでから 9 要素を抜き出していたのをやめ、
        /// 必要な 9 成分だけを直接作る。式は Matrix4x4.Rotation のものをそのまま転記しており
        /// (積・和・差の順序も同一)、値はビット単位で従来と同じ。</summary>
        public static Matrix3x3 FromQuat(Quat q)
        {
            float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
            float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
            float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
            // Matrix4x4 は転置格納 (Rᵀ) なので、行=Rの行 に揃えた並びで書く。
            return new Matrix3x3(
                new Vec3(1f - 2f * (yy + zz), 2f * (xy - wz),      2f * (xz + wy)),
                new Vec3(2f * (xy + wz),      1f - 2f * (xx + zz), 2f * (yz - wx)),
                new Vec3(2f * (xz - wy),      2f * (yz + wx),      1f - 2f * (xx + yy)));
        }

        /// <summary>★2026-09-07: Vec3 の添字プロパティ (範囲外 throw 付きの switch) を 3 回通していたのを
        /// フィールド直参照に置き換え。並べ替えるだけで演算は無いのでビット不変。</summary>
        public Vec3 Column(int i) => i switch
        {
            0 => new Vec3(Row0.x, Row1.x, Row2.x),
            1 => new Vec3(Row0.y, Row1.y, Row2.y),
            2 => new Vec3(Row0.z, Row1.z, Row2.z),
            _ => throw new ArgumentOutOfRangeException(nameof(i))
        };

        /// <summary>Bullet 2.75 btMatrix3x3::setRotation (btMatrix3x3.h:136) の移植。
        /// FromQuat と数学的には同じだが **式と丸めが違う**。
        /// ロック軸の限界判定は「変位が厳密に 0 かどうか」で行を作る/作らないが決まるため、
        /// この 1e-8 レベルの残差の有無がそのまま拘束の有無に化ける (タスク62)。</summary>
        public static Matrix3x3 FromQuatBullet(Quat q)
        {
            float d = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
            float sc = 2f / d;
            float xs = q.x * sc, ys = q.y * sc, zs = q.z * sc;
            float wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
            float xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
            float yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
            return new Matrix3x3(
                new Vec3(1f - (yy + zz), xy - wz,        xz + wy),
                new Vec3(xy + wz,        1f - (xx + zz), yz - wx),
                new Vec3(xz - wy,        yz + wx,        1f - (xx + yy)));
        }

        public static Vec3 operator *(Matrix3x3 m, Vec3 v) =>
            new(m.Row0.Dot(v), m.Row1.Dot(v), m.Row2.Dot(v));

        public static Matrix3x3 operator *(Matrix3x3 a, Matrix3x3 b)
        {
            var c0 = b.Column(0); var c1 = b.Column(1); var c2 = b.Column(2);
            return new Matrix3x3(
                new Vec3(a.Row0.Dot(c0), a.Row0.Dot(c1), a.Row0.Dot(c2)),
                new Vec3(a.Row1.Dot(c0), a.Row1.Dot(c1), a.Row1.Dot(c2)),
                new Vec3(a.Row2.Dot(c0), a.Row2.Dot(c1), a.Row2.Dot(c2)));
        }

        public Matrix3x3 Transposed() =>
            new(Column(0), Column(1), Column(2));

        /// <summary>Bullet 2.75 btMatrix3x3::inverse() の移植 (btMatrix3x3.h:536)。
        /// **余因子行列 ÷ 行列式** であって転置ではない。直交行列なら数学的には転置と同じだが、
        /// 浮動小数の値が違う: 転置は元の成分をそのまま並べ替えるので厳密な 0 が保たれるのに対し、
        /// 余因子は積と差を通るので厳密な 0 にならない。
        ///
        /// ★これが効くのは Bullet がロック軸の変位 (testLimitValue に食わせる値) を
        ///   calculateLinearInfo / calculateAngleInfo でこの逆行列から作っているため。
        ///   転置で代用するとバインド姿勢のような「誤差が厳密に 0」の姿勢で
        ///   testLimitValue が 0 (=行不要) を返しすぎ、拘束そのものが消える。
        ///   実測 (タスク61): バインド姿勢の行数が Bullet 95 に対し当方 58 まで落ち、
        ///   髪が1ステップで自由落下 (g·dt² の全量 0.0272) した。</summary>
        public Matrix3x3 BulletInverse()
        {
            // ★2026-09-07: 従来は `self.R(r1)[c1]` で読んでいた。R(int) は Vec3 を **値で返す** ので
            //   1 呼び出しごとに 12 バイトのコピーが起き、さらに Vec3 の添字 (範囲外 throw 付き switch)
            //   を通る。Cofac 12 回 × 4 読みで 48 回ぶん。成分をローカルに展開すれば全部消える。
            //
            //   ★Cofac を **ローカル関数のまま残すこと**。ここが丸め点になっている。
            //     a*b - c*d の中間値は評価スタック上では float32 より高い精度を持ちうるが (ECMA-335)、
            //     戻り値の型が float なので **return で float32 へ丸められる**。
            //     式に展開して `(a*b - c*d) * sc` と書くとこの丸めが消え、結果が 1 ULP ずれる。
            //     実測: 剛体570・300ステップの全姿勢ダンプで 135,306 行が変化した (2026-09-07)。
            float m00 = Row0.x, m01 = Row0.y, m02 = Row0.z;
            float m10 = Row1.x, m11 = Row1.y, m12 = Row1.z;
            float m20 = Row2.x, m21 = Row2.y, m22 = Row2.z;
            float Cofac(float a, float b, float c, float d) => a * b - c * d;

            var co = new Vec3(Cofac(m11, m22, m12, m21),    // 旧 Cofac(1,1,2,2)
                              Cofac(m12, m20, m10, m22),    // 旧 Cofac(1,2,2,0)
                              Cofac(m10, m21, m11, m20));   // 旧 Cofac(1,0,2,1)
            float det = Row0.Dot(co);
            float sc = 1f / det;
            return new Matrix3x3(
                new Vec3(co.x * sc, Cofac(m02, m21, m01, m22) * sc, Cofac(m01, m12, m02, m11) * sc),
                new Vec3(co.y * sc, Cofac(m00, m22, m02, m20) * sc, Cofac(m02, m10, m00, m12) * sc),
                new Vec3(co.z * sc, Cofac(m01, m20, m00, m21) * sc, Cofac(m00, m11, m01, m10) * sc));
        }

        /// <summary>this * diag(scale) * this^T — basis に対角テンソルを回転適用。</summary>
        public Matrix3x3 Scaled(Vec3 scale)
        {
            // 行列 basis の各列を scale 倍して basis^T を乗算する形。
            // 慣性テンソルのワールド変換: R * I_local * R^T に使用。
            return this * Matrix3x3.Diagonal(scale) * Transposed();
        }
    }
}
