// ===========================================================================
// Unity Bullet 互換物理エンジン – PMX Reader
// PMX 2.0 / 2.1 バイナリを解析し、剛体 / Joint / SoftBody を取り出す。
// 物理に不要なセクション (頂点・面・材質・モーフ等) は正確にスキップする。
// ===========================================================================

using System;
using System.IO;
using System.Text;

namespace BulletPhysics.Pmx
{
    public sealed class PmxReader
    {
        private BinaryReader _r;

        // ヘッダ由来のインデックスサイズ / エンコード。
        private int _encoding;      // 0:UTF16 1:UTF8
        private int _addUv;
        private int _vertexIndexSize;
        private int _textureIndexSize;
        private int _materialIndexSize;
        private int _boneIndexSize;
        private int _morphIndexSize;
        private int _rigidIndexSize;

        public static PmxPhysicsModel LoadFile(string path)
        {
            using var fs = File.OpenRead(path);
            return new PmxReader().Read(fs);
        }

        public PmxPhysicsModel Read(Stream stream)
        {
            _r = new BinaryReader(stream);
            var model = new PmxPhysicsModel();

            ReadHeader(model);
            // モデル情報 (名前/コメント 4 つ)。
            model.ModelName = ReadText();
            model.ModelNameEn = ReadText();
            ReadText(); // comment
            ReadText(); // comment en

            SkipVertices();
            SkipFaces();
            SkipTextures();
            SkipMaterials();
            ReadBones(model);      // 名前/位置だけ拾う
            SkipMorphs();
            SkipDisplayFrames();

            ReadRigidBodies(model);
            ReadJoints(model);

            if (model.Version >= 2.1f && stream.Position < stream.Length)
                ReadSoftBodies(model);

            return model;
        }

        // --- ヘッダ ---
        private void ReadHeader(PmxPhysicsModel model)
        {
            var magic = _r.ReadBytes(4);
            if (magic[0] != 'P' || magic[1] != 'M' || magic[2] != 'X' || magic[3] != ' ')
                throw new InvalidDataException("PMX マジックナンバー不一致");

            model.Version = _r.ReadSingle();

            byte flagCount = _r.ReadByte();
            var flags = _r.ReadBytes(flagCount);
            _encoding = flags[0];
            _addUv = flags[1];
            _vertexIndexSize = flags[2];
            _textureIndexSize = flags[3];
            _materialIndexSize = flags[4];
            _boneIndexSize = flags[5];
            _morphIndexSize = flags[6];
            _rigidIndexSize = flags[7];
        }

        // --- プリミティブ ---
        private Vec3 ReadVec3() => new(_r.ReadSingle(), _r.ReadSingle(), _r.ReadSingle());

        private string ReadText()
        {
            int len = _r.ReadInt32();
            if (len <= 0) return string.Empty;
            var bytes = _r.ReadBytes(len);
            return _encoding == 0
                ? Encoding.Unicode.GetString(bytes)      // UTF-16LE
                : Encoding.UTF8.GetString(bytes);
        }

        /// <summary>符号付きインデックス (-1 = 非参照)。</summary>
        private int ReadIndex(int size)
        {
            return size switch
            {
                1 => _r.ReadSByte(),
                2 => _r.ReadInt16(),
                4 => _r.ReadInt32(),
                _ => throw new InvalidDataException($"不正なインデックスサイズ {size}")
            };
        }

        /// <summary>符号なしインデックス (頂点用)。</summary>
        private int ReadVertexIndex()
        {
            return _vertexIndexSize switch
            {
                1 => _r.ReadByte(),
                2 => _r.ReadUInt16(),
                4 => _r.ReadInt32(),
                _ => throw new InvalidDataException("不正な頂点インデックスサイズ")
            };
        }

        private void Skip(long bytes) => _r.BaseStream.Seek(bytes, SeekOrigin.Current);

        // --- 頂点 ---
        private void SkipVertices()
        {
            int count = _r.ReadInt32();
            for (int i = 0; i < count; i++)
            {
                Skip(12 + 12 + 8);          // pos + normal + uv
                Skip(16 * _addUv);          // 追加 UV
                byte weightType = _r.ReadByte();
                switch (weightType)
                {
                    case 0: Skip(_boneIndexSize); break;                       // BDEF1
                    case 1: Skip(_boneIndexSize * 2 + 4); break;              // BDEF2
                    case 2: Skip(_boneIndexSize * 4 + 16); break;             // BDEF4
                    case 3: Skip(_boneIndexSize * 2 + 4 + 36); break;         // SDEF
                    case 4: Skip(_boneIndexSize * 4 + 16); break;             // QDEF (2.1)
                    default: throw new InvalidDataException($"不正なウェイト形式 {weightType}");
                }
                Skip(4);                    // エッジ倍率
            }
        }

        private void SkipFaces()
        {
            int count = _r.ReadInt32();     // 総インデックス数
            Skip((long)count * _vertexIndexSize);
        }

        private void SkipTextures()
        {
            int count = _r.ReadInt32();
            for (int i = 0; i < count; i++) ReadText();
        }

        private void SkipMaterials()
        {
            int count = _r.ReadInt32();
            for (int i = 0; i < count; i++)
            {
                ReadText(); ReadText();                 // 名前
                Skip(16 + 12 + 4 + 12);                 // diffuse+specular+coef+ambient
                Skip(1);                                // 描画フラグ
                Skip(16 + 4);                           // エッジ色 + サイズ
                Skip(_textureIndexSize * 2);            // texture + sphere
                Skip(1);                                // sphere mode
                byte sharedToon = _r.ReadByte();
                Skip(sharedToon == 0 ? _textureIndexSize : 1);
                ReadText();                             // メモ
                Skip(4);                                // 面数
            }
        }

        // ボーンは名前/位置のみ収集 (剛体<->ボーン照合に使用)。
        private void ReadBones(PmxPhysicsModel model)
        {
            int count = _r.ReadInt32();
            for (int i = 0; i < count; i++)
            {
                model.BoneNames.Add(ReadText());        // 名前
                ReadText();                             // 名前英
                model.BonePositions.Add(ReadVec3());    // 位置
                model.BoneParents.Add(ReadIndex(_boneIndexSize));  // 親 (-1 = ルート)
                model.BoneDeformLayers.Add(_r.ReadInt32());        // 変形階層
                ushort flags = _r.ReadUInt16();

                bool connectByBone = (flags & 0x0001) != 0;
                Skip(connectByBone ? _boneIndexSize : 12);

                if ((flags & 0x0100) != 0 || (flags & 0x0200) != 0) // 回転/移動付与
                    Skip(_boneIndexSize + 4);

                if ((flags & 0x0400) != 0) Skip(12);    // 軸固定
                if ((flags & 0x0800) != 0) Skip(24);    // ローカル軸 (X,Z)
                if ((flags & 0x2000) != 0) Skip(4);     // 外部親変形 Key

                if ((flags & 0x0020) != 0)              // IK
                {
                    Skip(_boneIndexSize);               // target
                    Skip(4 + 4);                        // ループ回数 + 制限角
                    int links = _r.ReadInt32();
                    for (int l = 0; l < links; l++)
                    {
                        Skip(_boneIndexSize);
                        byte limit = _r.ReadByte();
                        if (limit == 1) Skip(24);       // 下限+上限
                    }
                }
            }
        }

        private void SkipMorphs()
        {
            int count = _r.ReadInt32();
            for (int i = 0; i < count; i++)
            {
                ReadText(); ReadText();                 // 名前
                Skip(1);                                // 操作パネル
                byte type = _r.ReadByte();              // モーフ種類
                int offsets = _r.ReadInt32();
                for (int o = 0; o < offsets; o++)
                    SkipMorphOffset(type);
            }
        }

        private void SkipMorphOffset(byte type)
        {
            switch (type)
            {
                case 0: Skip(_morphIndexSize + 4); break;                 // グループ
                case 1: Skip(_vertexIndexSize + 12); break;              // 頂点
                case 2: Skip(_boneIndexSize + 12 + 16); break;          // ボーン
                case 3: case 4: case 5: case 6: case 7:                  // UV / 追加UV
                    Skip(_vertexIndexSize + 16); break;
                case 8: Skip(_materialIndexSize + 1 + 16 + 12 + 4 + 12 + 16 + 4 + 16 + 16 + 16); break; // 材質
                case 9: Skip(_morphIndexSize + 4); break;                // フリップ (2.1)
                case 10: Skip(_rigidIndexSize + 1 + 12 + 12); break;    // インパルス (2.1)
                default: throw new InvalidDataException($"不正なモーフ種類 {type}");
            }
        }

        private void SkipDisplayFrames()
        {
            int count = _r.ReadInt32();
            for (int i = 0; i < count; i++)
            {
                ReadText(); ReadText();                 // 枠名
                Skip(1);                                // 特殊枠フラグ
                int elems = _r.ReadInt32();
                for (int e = 0; e < elems; e++)
                {
                    byte target = _r.ReadByte();
                    Skip(target == 0 ? _boneIndexSize : _morphIndexSize);
                }
            }
        }

        // --- 剛体 ---
        private void ReadRigidBodies(PmxPhysicsModel model)
        {
            int count = _r.ReadInt32();
            for (int i = 0; i < count; i++)
            {
                var rb = new PmxRigidBody
                {
                    Name = ReadText(),
                    NameEn = ReadText(),
                    BoneIndex = ReadIndex(_boneIndexSize),
                    Group = _r.ReadByte(),
                    NonCollisionGroup = _r.ReadUInt16(),
                    ShapeType = _r.ReadByte(),
                    Size = ReadVec3(),
                    Position = ReadVec3(),
                    Rotation = ReadVec3(),
                    Mass = _r.ReadSingle(),
                    LinearDamping = _r.ReadSingle(),
                    AngularDamping = _r.ReadSingle(),
                    Restitution = _r.ReadSingle(),
                    Friction = _r.ReadSingle(),
                    PhysicsMode = _r.ReadByte(),
                };
                model.RigidBodies.Add(rb);
            }
        }

        // --- Joint ---
        private void ReadJoints(PmxPhysicsModel model)
        {
            int count = _r.ReadInt32();
            for (int i = 0; i < count; i++)
            {
                var j = new PmxJoint
                {
                    Name = ReadText(),
                    NameEn = ReadText(),
                    JointType = _r.ReadByte(),
                    RigidBodyAIndex = ReadIndex(_rigidIndexSize),
                    RigidBodyBIndex = ReadIndex(_rigidIndexSize),
                    Position = ReadVec3(),
                    Rotation = ReadVec3(),
                    LinearLowerLimit = ReadVec3(),
                    LinearUpperLimit = ReadVec3(),
                    AngularLowerLimit = ReadVec3(),
                    AngularUpperLimit = ReadVec3(),
                    SpringLinear = ReadVec3(),
                    SpringAngular = ReadVec3(),
                };
                model.Joints.Add(j);
            }
        }

        // --- SoftBody (2.1) ---
        private void ReadSoftBodies(PmxPhysicsModel model)
        {
            int count = _r.ReadInt32();
            for (int i = 0; i < count; i++)
            {
                var sb = new PmxSoftBody
                {
                    Name = ReadText(),
                    NameEn = ReadText(),
                    Shape = _r.ReadByte(),
                    MaterialIndex = ReadIndex(_materialIndexSize),
                    Group = _r.ReadByte(),
                    NonCollisionGroup = _r.ReadUInt16(),
                    Flags = _r.ReadByte(),
                    BLinkDistance = _r.ReadInt32(),
                    ClusterCount = _r.ReadInt32(),
                    TotalMass = _r.ReadSingle(),
                    CollisionMargin = _r.ReadSingle(),
                    AeroModel = _r.ReadInt32(),
                    VCF = _r.ReadSingle(), DP = _r.ReadSingle(), DG = _r.ReadSingle(),
                    LF = _r.ReadSingle(), PR = _r.ReadSingle(), VC = _r.ReadSingle(),
                    DF = _r.ReadSingle(), MT = _r.ReadSingle(), CHR = _r.ReadSingle(),
                    KHR = _r.ReadSingle(), SHR = _r.ReadSingle(), AHR = _r.ReadSingle(),
                    SRHR_CL = _r.ReadSingle(), SKHR_CL = _r.ReadSingle(), SSHR_CL = _r.ReadSingle(),
                    SR_SPLT_CL = _r.ReadSingle(), SK_SPLT_CL = _r.ReadSingle(), SS_SPLT_CL = _r.ReadSingle(),
                    V_IT = _r.ReadInt32(), P_IT = _r.ReadInt32(), D_IT = _r.ReadInt32(), C_IT = _r.ReadInt32(),
                    LST = _r.ReadSingle(), AST = _r.ReadSingle(), VST = _r.ReadSingle(),
                };

                int anchors = _r.ReadInt32();
                for (int a = 0; a < anchors; a++)
                {
                    sb.Anchors.Add(new PmxSoftBodyAnchor
                    {
                        RigidBodyIndex = ReadIndex(_rigidIndexSize),
                        VertexIndex = ReadVertexIndex(),
                        NearMode = _r.ReadByte() != 0,
                    });
                }

                int pins = _r.ReadInt32();
                for (int p = 0; p < pins; p++)
                    sb.PinVertices.Add(ReadVertexIndex());

                model.SoftBodies.Add(sb);
            }
        }
    }
}
