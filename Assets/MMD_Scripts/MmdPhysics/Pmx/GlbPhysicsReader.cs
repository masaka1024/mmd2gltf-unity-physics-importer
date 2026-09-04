// ===========================================================================
// GLB (glTF binary) の extras.mmd から PmxPhysicsModel を構築するリーダ。
// PmxReader (PMX 直読み) と並ぶ入力経路。物理置換(統合フェーズ)の土台。
//   - 剛体 : extras.mmd.rigidBodies (PMX raw) をそのままマップ
//   - Joint: extras.mmd.joints (PMX raw) をそのままマップ
//   - ボーン: glTF nodes から 名前 / 親(children階層) / 変形階層(extras.mmd.layer) /
//            raw PMX world 位置(ローカル並進をunitScaleで戻して階層合成)
//   - unitScale は extras.mmd.unitScale から読む (既定を仮定しない)
//   - physicsGltf(変換済みビュュー)は使わず raw を使う
// 当面 GLB(バイナリ)のみ対応 (.gltf JSON+外部bin は未対応)。読み取り専用・本体物理に非関与。
// ===========================================================================
using System;
using System.Collections.Generic;
using System.IO;

namespace BulletPhysics.Pmx
{
    public static class GlbPhysicsReader
    {
        // 座標: extras.mmd は raw PMX (MMD左手系, 未スケール)。glTF は cpos=(x,y,-z)*s。
        // よって glTF ローカル並進 lt から raw ローカルへ戻すのは (lt.x/s, lt.y/s, -lt.z/s)。
        // (バインドは回転恒等なので world = ローカルの単純合成でよい。)

        // 読み取り中に検出した異常(欠損・範囲外など)の警告。呼び出し側でログ表示できる。
        // 例外は投げず、可能な範囲で読み進めた結果を返す方針。
        public static PmxPhysicsModel LoadFile(string path) => LoadFile(path, out _, out _);

        public static PmxPhysicsModel LoadFile(string path, out float unitScale) => LoadFile(path, out unitScale, out _);

        public static PmxPhysicsModel LoadFile(string path, out float unitScale, out List<string> warnings)
            => LoadBytes(File.ReadAllBytes(path), out unitScale, out warnings);

        // バイト列(GLB)から直接読む (テスト・メモリ入力用)。
        public static PmxPhysicsModel LoadBytes(byte[] glb, out float unitScale, out List<string> warnings)
        {
            ParseGlb(glb, out var root, out _);
            return BuildModel(root, out unitScale, out warnings);
        }

        // ★ビルド同梱データ用の入口 (2026-09-04)。GLB コンテナでも **JSON チャンク単体** でも受ける。
        //   Unity のビルド (APK / exe) には Assets 内の .glb の「ソースファイル」は含まれないので、
        //   実行時に File.ReadAllBytes で読むことはできない。インポーターは GLB の JSON チャンク
        //   (BuildModel が使う extras.mmd と nodes/skins が全部ここに入っている) だけを
        //   .mmdphys.bytes として書き出し、TextAsset としてビルドへ同梱する。
        //   JSON チャンクは元 GLB のバイト列そのままなので、エディタで .glb を読んだ結果と同一。
        public static PmxPhysicsModel LoadBytesAuto(byte[] data, out float unitScale, out List<string> warnings)
        {
            if (data == null || data.Length < 4) throw new InvalidDataException("物理データが空です。");
            if (BitConverter.ToUInt32(data, 0) == 0x46546C67) return LoadBytes(data, out unitScale, out warnings);
            return LoadJson(Utf8NoBom(data), out unitScale, out warnings);
        }

        // glTF の JSON チャンク(テキスト)から読む。
        public static PmxPhysicsModel LoadJson(string json, out float unitScale, out List<string> warnings)
        {
            var root = MiniJson.Obj(MiniJson.Parse(json));
            if (root == null) throw new InvalidDataException("物理データを解釈できません (GLB でも glTF JSON でもない)。");
            return BuildModel(root, out unitScale, out warnings);
        }

        private static string Utf8NoBom(byte[] d)
        {
            int off = (d.Length >= 3 && d[0] == 0xEF && d[1] == 0xBB && d[2] == 0xBF) ? 3 : 0;
            return System.Text.Encoding.UTF8.GetString(d, off, d.Length - off);
        }

        // ibm(inverseBindMatrices)から算出した raw PMX ボーン world 位置 (検証の別経路)。
        public static Vec3[] BonePositionsFromIbm(string path)
        {
            ParseGlb(File.ReadAllBytes(path), out var root, out var bin);
            var mm = MiniJson.Obj(MiniJson.Get(MiniJson.Obj(MiniJson.Get(root, "extras")), "mmd"));
            float s = MiniJson.Flt(MiniJson.Get(mm, "unitScale"));
            var skin0 = MiniJson.Obj(MiniJson.Arr(MiniJson.Get(root, "skins"))[0]);
            int nb = MiniJson.Arr(MiniJson.Get(skin0, "joints")).Count;
            int ibmAcc = MiniJson.Int(MiniJson.Get(skin0, "inverseBindMatrices"));
            var mats = ReadFloatAccessor(root, bin, ibmAcc); // nb*16
            var pos = new Vec3[nb];
            for (int i = 0; i < nb; i++)
            {
                // 列優先 MAT4 の並進は要素 [12],[13],[14] = (-wp.x*s, -wp.y*s, wp.z*s)。
                float tx = mats[i * 16 + 12], ty = mats[i * 16 + 13], tz = mats[i * 16 + 14];
                pos[i] = new Vec3(-tx / s, -ty / s, tz / s);
            }
            return pos;
        }

        // ---- GLB コンテナ分解 (JSON チャンク + BIN チャンク) ----
        private static void ParseGlb(byte[] d, out Dictionary<string, object> root, out byte[] bin)
        {
            uint magic = BitConverter.ToUInt32(d, 0);
            if (magic != 0x46546C67) throw new InvalidDataException("GLB マジックが不正 (glTF ではない)");
            // uint version = BitConverter.ToUInt32(d, 4); uint length = BitConverter.ToUInt32(d, 8);
            int off = 12;
            string json = null; bin = null;
            while (off + 8 <= d.Length)
            {
                int clen = (int)BitConverter.ToUInt32(d, off);
                uint ctype = BitConverter.ToUInt32(d, off + 4);
                int cdata = off + 8;
                if (ctype == 0x4E4F534A)      // "JSON"
                    json = System.Text.Encoding.UTF8.GetString(d, cdata, clen);
                else if (ctype == 0x004E4942) // "BIN\0"
                {
                    bin = new byte[clen];
                    Array.Copy(d, cdata, bin, 0, clen);
                }
                off = cdata + clen;
                if ((clen & 3) != 0) off += 4 - (clen & 3); // 4バイト境界パディング
            }
            if (json == null) throw new InvalidDataException("GLB に JSON チャンクが無い");
            root = MiniJson.Obj(MiniJson.Parse(json));
        }

        // ---- extras.mmd → PmxPhysicsModel (防御的: 欠損/範囲外でも例外を投げず読み進める) ----
        private static PmxPhysicsModel BuildModel(Dictionary<string, object> root, out float unitScale, out List<string> warnings)
        {
            warnings = new List<string>();
            var model = new PmxPhysicsModel();
            var mm = MiniJson.Obj(MiniJson.Get(MiniJson.Obj(MiniJson.Get(root, "extras")), "mmd"));
            if (mm == null) { warnings.Add("extras.mmd が無い (物理情報なし)。ボーンのみ読み進める。"); mm = new Dictionary<string, object>(); }
            float scale = MiniJson.Flt(MiniJson.Get(mm, "unitScale"));
            if (scale <= 0f) { if (mm.Count > 0) warnings.Add($"unitScale が不正 ({scale})。1.0 として続行。"); scale = 1f; }
            unitScale = scale;

            // --- ボーン (nodes + skins[0].joints) ---
            var nodes = MiniJson.Arr(MiniJson.Get(root, "nodes"));
            var skins = MiniJson.Arr(MiniJson.Get(root, "skins"));
            var skin0 = (skins != null && skins.Count > 0) ? MiniJson.Obj(skins[0]) : null;
            var jointNodes = MiniJson.Arr(MiniJson.Get(skin0, "joints"));
            int nb = (jointNodes != null && nodes != null) ? jointNodes.Count : 0;
            if (nodes == null || skin0 == null || jointNodes == null)
                warnings.Add("nodes/skins/joints が無いためボーン0件で続行。");

            var localRaw = new Vec3[nb];       // raw PMX ローカル並進
            var parent = new int[nb];
            for (int i = 0; i < nb; i++) parent[i] = -1;

            for (int i = 0; i < nb; i++)
            {
                int ni = MiniJson.Int(jointNodes[i]);
                var node = (ni >= 0 && ni < nodes.Count) ? MiniJson.Obj(nodes[ni]) : null;
                if (node == null) { warnings.Add($"ボーン{i} のノードindex {ni} が範囲外。"); model.BoneNames.Add("bone_" + i); model.BoneDeformLayers.Add(0); continue; }
                model.BoneNames.Add(MiniJson.Str(MiniJson.Get(node, "name")) ?? ("bone_" + i));
                var lt = Vec3FromArr(MiniJson.Get(node, "translation"));
                localRaw[i] = new Vec3(lt.x / scale, lt.y / scale, -lt.z / scale); // glTF→raw
                var ex = MiniJson.Obj(MiniJson.Get(MiniJson.Obj(MiniJson.Get(node, "extras")), "mmd"));
                model.BoneDeformLayers.Add(ex != null ? MiniJson.Int(MiniJson.Get(ex, "layer")) : 0);
            }
            if (nodes != null)
            // 親: 各ノードの children から。子ノードindexが 0..nb-1 のボーンなら親を設定。
            //     親ノードが 0..nb-1 のボーンなら親=そのindex、そうでなければ(スケルトンroot等)=-1。
            for (int j = 0; j < nodes.Count; j++)
            {
                var ch = MiniJson.Arr(MiniJson.Get(MiniJson.Obj(nodes[j]), "children"));
                if (ch == null) continue;
                foreach (var c in ch)
                {
                    int ci = MiniJson.Int(c);
                    if (ci >= 0 && ci < nb) parent[ci] = (j < nb) ? j : -1;
                }
            }
            for (int i = 0; i < nb; i++) model.BoneParents.Add(parent[i]);

            // world 位置 = ローカル並進の階層合成 (バインド回転恒等)。親を先に確定。
            var world = new Vec3?[nb];
            Vec3 World(int i)
            {
                if (world[i].HasValue) return world[i].Value;
                int p = parent[i];
                Vec3 w = (p >= 0 && p < nb) ? localRaw[i] + World(p) : localRaw[i];
                world[i] = w; return w;
            }
            for (int i = 0; i < nb; i++) model.BonePositions.Add(World(i));

            // --- 剛体 (raw) ---
            var rbs = MiniJson.Arr(MiniJson.Get(mm, "rigidBodies"));
            if (rbs != null)
                foreach (var o in rbs)
                {
                    var r = MiniJson.Obj(o);
                    model.RigidBodies.Add(new PmxRigidBody
                    {
                        Name = MiniJson.Str(MiniJson.Get(r, "name")) ?? "",
                        NameEn = MiniJson.Str(MiniJson.Get(r, "name_en")) ?? "",
                        BoneIndex = MiniJson.Int(MiniJson.Get(r, "bone")),
                        Group = (byte)MiniJson.Int(MiniJson.Get(r, "group")),
                        NonCollisionGroup = (ushort)MiniJson.Int(MiniJson.Get(r, "no_collision_mask")),
                        ShapeType = (byte)MiniJson.Int(MiniJson.Get(r, "shape")),
                        Size = Vec3FromArr(MiniJson.Get(r, "size")),
                        Position = Vec3FromArr(MiniJson.Get(r, "pos")),
                        Rotation = Vec3FromArr(MiniJson.Get(r, "rot")),
                        Mass = MiniJson.Flt(MiniJson.Get(r, "mass")),
                        LinearDamping = MiniJson.Flt(MiniJson.Get(r, "linear_damping")),
                        AngularDamping = MiniJson.Flt(MiniJson.Get(r, "angular_damping")),
                        Restitution = MiniJson.Flt(MiniJson.Get(r, "restitution")),
                        Friction = MiniJson.Flt(MiniJson.Get(r, "friction")),
                        PhysicsMode = (byte)MiniJson.Int(MiniJson.Get(r, "mode")),
                    });
                }

            // --- Joint (raw) ---
            var jts = MiniJson.Arr(MiniJson.Get(mm, "joints"));
            if (jts != null)
                foreach (var o in jts)
                {
                    var j = MiniJson.Obj(o);
                    model.Joints.Add(new PmxJoint
                    {
                        Name = MiniJson.Str(MiniJson.Get(j, "name")) ?? "",
                        NameEn = MiniJson.Str(MiniJson.Get(j, "name_en")) ?? "",
                        JointType = (byte)MiniJson.Int(MiniJson.Get(j, "type")),
                        RigidBodyAIndex = MiniJson.Int(MiniJson.Get(j, "rigid_a")),
                        RigidBodyBIndex = MiniJson.Int(MiniJson.Get(j, "rigid_b")),
                        Position = Vec3FromArr(MiniJson.Get(j, "pos")),
                        Rotation = Vec3FromArr(MiniJson.Get(j, "rot")),
                        LinearLowerLimit = Vec3FromArr(MiniJson.Get(j, "pos_min")),
                        LinearUpperLimit = Vec3FromArr(MiniJson.Get(j, "pos_max")),
                        AngularLowerLimit = Vec3FromArr(MiniJson.Get(j, "rot_min")),
                        AngularUpperLimit = Vec3FromArr(MiniJson.Get(j, "rot_max")),
                        SpringLinear = Vec3FromArr(MiniJson.Get(j, "spring_pos")),
                        SpringAngular = Vec3FromArr(MiniJson.Get(j, "spring_rot")),
                    });
                }

            // --- 参照整合性の検査 (修正はせず警告のみ。PmxPhysicsBuilder 側が範囲外を許容する) ---
            for (int i = 0; i < model.RigidBodies.Count; i++)
                if (model.RigidBodies[i].BoneIndex >= model.BoneNames.Count)
                    warnings.Add($"剛体 '{model.RigidBodies[i].Name}' の関連ボーンindex {model.RigidBodies[i].BoneIndex} が範囲外(ボーン数{model.BoneNames.Count})。バインド位置扱い。");
            for (int i = 0; i < model.Joints.Count; i++)
            {
                var jj = model.Joints[i]; int rc = model.RigidBodies.Count;
                if (jj.RigidBodyAIndex < 0 || jj.RigidBodyAIndex >= rc || jj.RigidBodyBIndex < 0 || jj.RigidBodyBIndex >= rc)
                    warnings.Add($"Joint '{jj.Name}' が存在しない剛体を参照(A={jj.RigidBodyAIndex} B={jj.RigidBodyBIndex}, 剛体数{rc})。構築時にスキップされる。");
            }

            return model;
        }

        private static Vec3 Vec3FromArr(object o)
        {
            var a = MiniJson.Arr(o);
            if (a == null || a.Count < 3) return Vec3.Zero;
            return new Vec3(MiniJson.Flt(a[0]), MiniJson.Flt(a[1]), MiniJson.Flt(a[2]));
        }

        // accessor(FLOAT)を BIN から読み出す (bufferView.byteOffset + accessor.byteOffset)。
        private static float[] ReadFloatAccessor(Dictionary<string, object> root, byte[] bin, int accIdx)
        {
            var acc = MiniJson.Obj(MiniJson.Arr(MiniJson.Get(root, "accessors"))[accIdx]);
            int count = MiniJson.Int(MiniJson.Get(acc, "count"));
            string type = MiniJson.Str(MiniJson.Get(acc, "type"));
            int comps = type == "MAT4" ? 16 : type == "VEC3" ? 3 : type == "VEC4" ? 4 : type == "SCALAR" ? 1 : 16;
            int accOff = acc.ContainsKey("byteOffset") ? MiniJson.Int(MiniJson.Get(acc, "byteOffset")) : 0;
            var bv = MiniJson.Obj(MiniJson.Arr(MiniJson.Get(root, "bufferViews"))[MiniJson.Int(MiniJson.Get(acc, "bufferView"))]);
            int bvOff = bv.ContainsKey("byteOffset") ? MiniJson.Int(MiniJson.Get(bv, "byteOffset")) : 0;
            int baseOff = bvOff + accOff;
            var outp = new float[count * comps];
            for (int i = 0; i < outp.Length; i++) outp[i] = BitConverter.ToSingle(bin, baseOff + i * 4);
            return outp;
        }
    }
}
