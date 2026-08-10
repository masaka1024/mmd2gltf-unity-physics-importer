// ===========================================================================
// ボーン姿勢CSVソース (UnityEngine 非依存)。
// MMDでベイクしたVMDから書き出した「ワールド絶対姿勢」CSVを読み、フレーム×ボーンで返す。
//   列: frame,boneName,posX,posY,posZ,quatX,quatY,quatZ,quatW (ヘッダ行あり, UTF-8)
//   座標系・単位は PMX ネイティブ (エンジン内部と同一。変換なし)。
// ※ scratchpad の BoneCheck.BoneCsv と同一方式だが、Unity 側(Assets)から使うため独立に置く。
//   UnityEngine を参照しないので、Unity 実行時にも検証ハーネスにも同じくコンパイルできる。
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace BulletPhysics.Unity
{
    public sealed class BonePoseCsvSource
    {
        private readonly Dictionary<string, int> _col = new();   // boneName -> 列index
        private RigidTransform[][] _pose;                        // [frame][col]
        private bool[][] _present;                               // [frame][col]

        public int FrameCount { get; private set; }
        public int MaxFrame => FrameCount - 1;
        public IReadOnlyCollection<string> BoneNames => _col.Keys;
        public int BoneCount => _col.Count;

        public static BonePoseCsvSource Load(string path)
        {
            if (string.IsNullOrEmpty(path) || !File.Exists(path)) return null;
            var csv = new BonePoseCsvSource();
            csv.ReadAll(path);
            return csv;
        }

        private void ReadAll(string path)
        {
            int maxFrame = -1;
            var rows = new List<(int frame, string bone, RigidTransform xf)>(310000);
            bool header = true;
            foreach (var line in File.ReadLines(path))
            {
                if (header) { header = false; continue; }
                if (line.Length == 0) continue;
                var s = line.Split(',');
                if (s.Length < 9) continue;
                int frame = int.Parse(s[0], CultureInfo.InvariantCulture);
                string bone = s[1];
                var pos = new Vec3(F(s[2]), F(s[3]), F(s[4]));
                // クォータニオン正規化 (VMD由来float32の |q| ずれを吸収し、後段 acos の見かけ誤差を抑える)。
                var q = new Quat(F(s[5]), F(s[6]), F(s[7]), F(s[8])).Normalized;
                if (!_col.ContainsKey(bone)) _col[bone] = _col.Count;
                if (frame > maxFrame) maxFrame = frame;
                rows.Add((frame, bone, new RigidTransform(q, pos)));
            }
            FrameCount = maxFrame + 1;
            int cols = _col.Count;

            _pose = new RigidTransform[FrameCount][];
            _present = new bool[FrameCount][];
            for (int f = 0; f < FrameCount; f++)
            {
                _pose[f] = new RigidTransform[cols];
                _present[f] = new bool[cols];
            }
            foreach (var (frame, bone, xf) in rows)
            {
                int c = _col[bone];
                _pose[frame][c] = xf;
                _present[frame][c] = true;
            }
        }

        private static float F(string s) => float.Parse(s, CultureInfo.InvariantCulture);

        public bool HasBone(string bone) => _col.ContainsKey(bone);

        /// <summary>指定フレーム・ボーンのワールド姿勢 (PMXネイティブ)。無ければ false。</summary>
        public bool TryGet(int frame, string bone, out RigidTransform xf)
        {
            xf = RigidTransform.Identity;
            if (frame < 0 || frame >= FrameCount) return false;
            if (!_col.TryGetValue(bone, out int c)) return false;
            if (!_present[frame][c]) return false;
            xf = _pose[frame][c];
            return true;
        }
    }
}
