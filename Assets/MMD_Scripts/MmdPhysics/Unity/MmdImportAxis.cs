// ===========================================================================
// 取り込み経路の軸規約 (Issue #10: UniGLTF 必須を外す)
//
// なぜ要るのか:
//   mmd2gltf は PMX→glTF で Z を反転して書き出す。UniGLTF は glTF→Unity で再び Z を反転するので
//   相殺し、Unity のボーン座標はそのまま PMX ネイティブ座標になる (= 恒等)。
//   一方 Unity 標準の glTF インポーター (glTFast) は Z ではなく X を反転するため相殺せず、
//   スケルトンは PMX に対して Y軸180° 回った状態で取り込まれる。
//
//   取り込み時の軸変換 S は姿勢に S·T·S (共役) の形で掛かる。位置は符号が変わるが、
//   バインド時の回転は恒等のまま残る。剛体のボーンからのオフセットは PMX 空間で求めた値を
//   Unity 側のボーン回転にそのまま掛けるので、境界を恒等に固定したままだとオフセットだけが
//   Y軸180° 回った側に出る (髪・スカートが体の正面へ回り込む)。
//   境界で同じ S による共役変換を掛ければ剛体は正しい側に戻る。下の 4 候補はいずれも Y 軸を
//   保つので、重力もシミュレーションも PMX 空間内では変わらず、**物理結果は不変**のまま
//   Unity 側だけが整合する。
//
//   | 規約          | 位置 (x,y,z) | 回転 (x,y,z,w) |
//   |---------------|--------------|----------------|
//   | 一致 (UniGLTF) | そのまま     | そのまま        |
//   | Y軸180°(glTFast)| (−x, y, −z) | (−x, y, −z, w) |
//   | Z鏡映          | (x, y, −z)   | (−x, −y, z, w) |
//   | X鏡映          | (−x, y, z)   | (x, −y, −z, w) |
//
//   回転の式: S = diag(sx, 1, sz) による共役は、回転軸を擬ベクトルとして写す
//   (axis' = det(S)·S·axis、角度は不変)。よって q' = (det·sx·x, det·y, det·sz·z, w)。
//   いずれも自己逆写像 (S·S = I) なので、Unity→PMX と PMX→Unity は同じ関数で済む。
// ===========================================================================

using System.Collections.Generic;
using UnityEngine;

namespace BulletPhysics.Unity
{
    /// <summary>GLB をどのインポーターで取り込んだかで決まる、Unity ボーン座標と PMX 座標の関係。</summary>
    public enum ImportAxisConvention
    {
        /// <summary>未確定。起動時にボーン配置から自動判定する (配線時に確定値を保存するのが推奨)。</summary>
        Auto = 0,
        /// <summary>一致。UniGLTF (既定の Reverse Axis = Z) で取り込んだ状態。</summary>
        Identity = 1,
        /// <summary>Y軸180°回転。Unity 標準の glTF インポーター (glTFast) 等、X を反転する取り込み。</summary>
        RotateY180 = 2,
        /// <summary>Z鏡映。境界に余分な Z 反転が残っている状態。</summary>
        MirrorZ = 3,
        /// <summary>X鏡映。境界に余分な X 反転が残っている状態。</summary>
        MirrorX = 4,
    }

    /// <summary>軸規約ごとの座標変換・自動判定・ボーン解決。ランタイムとエディタ拡張の共通部品。</summary>
    public static class MmdImportAxis
    {
        // 判定候補 (Auto を除く)。並びと符号は旧 CheckImportConventionCore と同一。
        private static readonly ImportAxisConvention[] Candidates =
        {
            ImportAxisConvention.Identity, ImportAxisConvention.RotateY180,
            ImportAxisConvention.MirrorZ, ImportAxisConvention.MirrorX,
        };

        /// <summary>規約の表示名 (ログ用)。</summary>
        public static string Label(ImportAxisConvention c)
        {
            switch (c)
            {
                case ImportAxisConvention.Identity: return "一致";
                case ImportAxisConvention.RotateY180: return "Y軸180度回転";
                case ImportAxisConvention.MirrorZ: return "Z鏡映";
                case ImportAxisConvention.MirrorX: return "X鏡映";
                default: return "自動";
            }
        }

        /// <summary>S = diag(sx, 1, sz) の符号。Auto は Identity と同じ扱い。</summary>
        public static void Signs(ImportAxisConvention c, out float sx, out float sz)
        {
            switch (c)
            {
                case ImportAxisConvention.RotateY180: sx = -1f; sz = -1f; break;
                case ImportAxisConvention.MirrorZ: sx = 1f; sz = -1f; break;
                case ImportAxisConvention.MirrorX: sx = -1f; sz = 1f; break;
                default: sx = 1f; sz = 1f; break;
            }
        }

        private static bool IsIdentity(ImportAxisConvention c) =>
            c == ImportAxisConvention.Identity || c == ImportAxisConvention.Auto;

        // ── 位置 (自己逆) ──
        public static Vec3 ApplyPos(ImportAxisConvention c, Vec3 v)
        {
            if (IsIdentity(c)) return v;
            Signs(c, out float sx, out float sz);
            return new Vec3(sx * v.x, v.y, sz * v.z);
        }

        public static Vector3 ApplyPos(ImportAxisConvention c, Vector3 v)
        {
            if (IsIdentity(c)) return v;
            Signs(c, out float sx, out float sz);
            return new Vector3(sx * v.x, v.y, sz * v.z);
        }

        // ── 回転 (共役。自己逆) ──
        public static Quat ApplyRot(ImportAxisConvention c, Quat q)
        {
            if (IsIdentity(c)) return q;
            Signs(c, out float sx, out float sz);
            float det = sx * sz;
            return new Quat(det * sx * q.x, det * q.y, det * sz * q.z, q.w);
        }

        public static Quaternion ApplyRot(ImportAxisConvention c, Quaternion q)
        {
            if (IsIdentity(c)) return q;
            Signs(c, out float sx, out float sz);
            float det = sx * sz;
            return new Quaternion(det * sx * q.x, det * q.y, det * sz * q.z, q.w);
        }

        // ─────────────────────────────────────────────
        //  自動判定
        // ─────────────────────────────────────────────

        /// <summary>判定結果。</summary>
        public struct Detection
        {
            public ImportAxisConvention Best;
            public float BestRms, IdentityRms, Tolerance;
            public int Resolved, Total, Unresolved;
            /// <summary>判定に使えるだけボーンが解決できたか。</summary>
            public bool Enough => Resolved >= 4;
            /// <summary>最良候補の残差が許容内か (= Best を信用してよいか)。
            /// Play 中・ポーズ付きで保存されたプレハブでは false になりうる。</summary>
            public bool Reliable => Enough && BestRms <= Tolerance;

            public string Summary =>
                $"ボーン {Resolved}/{Total}" + (Unresolved > 0 ? $" (未解決 {Unresolved})" : "") +
                $" 残差RMS: 一致={IdentityRms:F4} / 最良={Label(Best)}={BestRms:F4} / 許容={Tolerance:F4} (PMX単位)";
        }

        /// <summary>ボーンの配置を PMX バインド位置と突き合わせ、4 候補の軸変換のうち最も合うものを選ぶ。
        /// 判定はモデルのシーン配置に影響されない: ModelRoot のローカルへ落として並進・回転・スケールを
        /// 除去し、さらに重心を引いてから比較する。**バインド姿勢 (Play 前) で呼ぶこと。**</summary>
        public static Detection Detect(Transform modelRoot, Transform[] bones, IList<Vec3> bindPositions, float unitScale)
        {
            var r = new Detection { Best = ImportAxisConvention.Identity, BestRms = float.MaxValue };
            if (modelRoot == null || bones == null || bindPositions == null) return r;
            if (unitScale <= 0f) unitScale = 1f;

            int n = bones.Length < bindPositions.Count ? bones.Length : bindPositions.Count;
            r.Total = n;
            var ux = new float[n]; var uy = new float[n]; var uz = new float[n];
            var px = new float[n]; var py = new float[n]; var pz = new float[n];
            int m = 0;
            for (int i = 0; i < n; i++)
            {
                var tr = bones[i];
                if (tr == null) { r.Unresolved++; continue; }
                var l = modelRoot.InverseTransformPoint(tr.position);
                ux[m] = l.x / unitScale; uy[m] = l.y / unitScale; uz[m] = l.z / unitScale;
                var p = bindPositions[i];
                px[m] = p.x; py[m] = p.y; pz[m] = p.z;
                m++;
            }
            r.Resolved = m;
            if (m < 4) return r;

            float cux = 0, cuy = 0, cuz = 0, cpx = 0, cpy = 0, cpz = 0;
            for (int i = 0; i < m; i++) { cux += ux[i]; cuy += uy[i]; cuz += uz[i]; cpx += px[i]; cpy += py[i]; cpz += pz[i]; }
            cux /= m; cuy /= m; cuz /= m; cpx /= m; cpy /= m; cpz /= m;

            for (int c = 0; c < Candidates.Length; c++)
            {
                Signs(Candidates[c], out float sx, out float sz);
                double acc = 0;
                for (int i = 0; i < m; i++)
                {
                    float dx = (ux[i] - cux) - sx * (px[i] - cpx);
                    float dy = (uy[i] - cuy) - (py[i] - cpy);
                    float dz = (uz[i] - cuz) - sz * (pz[i] - cpz);
                    acc += dx * dx + dy * dy + dz * dz;
                }
                float rms = (float)System.Math.Sqrt(acc / m);
                if (c == 0) r.IdentityRms = rms;
                if (rms < r.BestRms) { r.BestRms = rms; r.Best = Candidates[c]; }
            }

            // 許容量はモデルの広がりに対する相対値 (単位や体格に依存しないため)。
            double sacc = 0;
            for (int i = 0; i < m; i++)
            { float dx = px[i] - cpx, dy = py[i] - cpy, dz = pz[i] - cpz; sacc += dx * dx + dy * dy + dz * dz; }
            float spread = (float)System.Math.Sqrt(sacc / m);
            r.Tolerance = 0.02f * (spread > 1e-3f ? spread : 1f);
            return r;
        }

        // ─────────────────────────────────────────────
        //  ボーン解決 (名前依存を外す)
        // ─────────────────────────────────────────────

        /// <summary>BoneIndex → Transform を解決する。
        /// GLB 由来のボーン順は skins[0].joints 順なので、同じ数のボーンを持つ SkinnedMeshRenderer.bones を
        /// そのまま使う (glTFast は重複ノード名にサフィックスを付けるため名前照合が壊れる)。
        /// skin 順の妥当性は、名前が一意に決まるボーンで突き合わせて確かめる。1 本でも食い違えば
        /// skin 順は使わず、従来どおりの名前照合に戻る。</summary>
        /// <param name="includeInactive">名前照合で非アクティブも含めるか (呼び出し元の従来挙動に合わせる)</param>
        /// <param name="lastWinsOnDuplicate">同名が複数あるとき後勝ちにするか (同上)</param>
        /// <param name="method">どちらで解決したか (ログ用)</param>
        public static Transform[] ResolveBones(Transform root, IList<string> boneNames,
                                               bool includeInactive, bool lastWinsOnDuplicate,
                                               out string method)
        {
            int count = boneNames != null ? boneNames.Count : 0;
            var result = new Transform[count];
            method = "なし";
            if (root == null || count == 0) return result;

            // 名前照合 (従来経路) と、名前の出現回数。
            var byName = new Dictionary<string, Transform>();
            var nameCount = new Dictionary<string, int>();
            foreach (var t in root.GetComponentsInChildren<Transform>(includeInactive))
            {
                nameCount[t.name] = nameCount.TryGetValue(t.name, out int k) ? k + 1 : 1;
                if (lastWinsOnDuplicate || !byName.ContainsKey(t.name)) byName[t.name] = t;
            }

            Transform[] skinBones = FindSkinBones(root, boneNames, byName, nameCount, out int agreed);
            if (skinBones != null)
            {
                int filled = 0;
                for (int i = 0; i < count; i++)
                {
                    var t = skinBones[i];
                    if (t == null && byName.TryGetValue(boneNames[i], out var nt)) { t = nt; filled++; }
                    result[i] = t;
                }
                method = $"skin順 (名前で照合できた {agreed} 本と一致" + (filled > 0 ? $"、空き {filled} 本は名前で補完" : "") + ")";
                return result;
            }

            for (int i = 0; i < count; i++)
                if (byName.TryGetValue(boneNames[i], out var tr)) result[i] = tr;
            method = "名前照合";
            return result;
        }

        private static Transform[] FindSkinBones(Transform root, IList<string> boneNames,
                                                 Dictionary<string, Transform> byName,
                                                 Dictionary<string, int> nameCount, out int agreed)
        {
            agreed = 0;
            // 物理データ側で同名が複数あるボーンは、Unity 側で改名されていると名前が「一意」に見えても
            // 別のボーンを指しうる (例: glTFast が 2 本目を「髪_1」に改名 → 「髪」は 1 本だけになる)。
            var modelNameCount = new Dictionary<string, int>();
            foreach (var nm in boneNames)
                modelNameCount[nm] = modelNameCount.TryGetValue(nm, out int k) ? k + 1 : 1;

            foreach (var smr in root.GetComponentsInChildren<SkinnedMeshRenderer>(true))
            {
                var bones = smr.bones;
                if (bones == null || bones.Length != boneNames.Count) continue;

                int ok = 0, bad = 0;
                for (int i = 0; i < bones.Length; i++)
                {
                    string name = boneNames[i];
                    // 名前が両側で一意に決まるボーンだけを証拠に使う (重複名・改名されたものは判定に使わない)。
                    if (modelNameCount[name] != 1) continue;
                    if (!nameCount.TryGetValue(name, out int c) || c != 1) continue;
                    if (bones[i] == null) continue;
                    if (bones[i] == byName[name]) ok++; else bad++;
                }
                if (bad == 0 && ok > 0) { agreed = ok; return bones; }
            }
            return null;
        }
    }
}
