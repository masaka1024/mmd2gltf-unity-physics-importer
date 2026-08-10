// ===========================================================================
// Unity Bullet 互換物理エンジン – PMX 物理データ構造
// PMX ファイルから抽出した剛体 / Joint / SoftBody の生パラメータ。
// ===========================================================================

using System.Collections.Generic;

namespace BulletPhysics.Pmx
{
    /// <summary>PMX 剛体レコード (仕様 ●剛体)。</summary>
    public sealed class PmxRigidBody
    {
        public string Name = "";
        public string NameEn = "";
        public int BoneIndex = -1;
        public byte Group;
        public ushort NonCollisionGroup;
        public byte ShapeType;          // 0:球 1:箱 2:カプセル
        public Vec3 Size;               // 半径/半径長
        public Vec3 Position;           // ワールド位置
        public Vec3 Rotation;           // ラジアン
        public float Mass;
        public float LinearDamping;
        public float AngularDamping;
        public float Restitution;
        public float Friction;
        public byte PhysicsMode;        // 0:追従 1:物理 2:物理+Bone合わせ
    }

    /// <summary>PMX Joint レコード (仕様 ●Joint)。</summary>
    public sealed class PmxJoint
    {
        public string Name = "";
        public string NameEn = "";
        public byte JointType;          // 0..5
        public int RigidBodyAIndex = -1;
        public int RigidBodyBIndex = -1;
        public Vec3 Position;
        public Vec3 Rotation;           // ラジアン
        public Vec3 LinearLowerLimit;
        public Vec3 LinearUpperLimit;
        public Vec3 AngularLowerLimit;
        public Vec3 AngularUpperLimit;
        public Vec3 SpringLinear;
        public Vec3 SpringAngular;
    }

    /// <summary>PMX SoftBody アンカー。</summary>
    public struct PmxSoftBodyAnchor
    {
        public int RigidBodyIndex;
        public int VertexIndex;
        public bool NearMode;
    }

    /// <summary>PMX SoftBody レコード (仕様 ●SoftBody 2.1)。</summary>
    public sealed class PmxSoftBody
    {
        public string Name = "";
        public string NameEn = "";
        public byte Shape;              // 0:TriMesh 1:Rope
        public int MaterialIndex = -1;
        public byte Group;
        public ushort NonCollisionGroup;
        public byte Flags;              // 0x01:B-Link 0x02:Cluster 0x04:LinkCross
        public int BLinkDistance;
        public int ClusterCount;
        public float TotalMass;
        public float CollisionMargin;
        public int AeroModel;

        // config
        public float VCF, DP, DG, LF, PR, VC, DF, MT, CHR, KHR, SHR, AHR;
        // cluster
        public float SRHR_CL, SKHR_CL, SSHR_CL, SR_SPLT_CL, SK_SPLT_CL, SS_SPLT_CL;
        // iteration
        public int V_IT, P_IT, D_IT, C_IT;
        // material
        public float LST, AST, VST;

        public readonly List<PmxSoftBodyAnchor> Anchors = new();
        public readonly List<int> PinVertices = new();
    }

    /// <summary>PMX から抽出した物理関連データ一式。</summary>
    public sealed class PmxPhysicsModel
    {
        public string ModelName = "";
        public string ModelNameEn = "";
        public float Version;

        public readonly List<PmxRigidBody> RigidBodies = new();
        public readonly List<PmxJoint> Joints = new();
        public readonly List<PmxSoftBody> SoftBodies = new();

        // ボーン名 (剛体<->ボーン紐付け用に最小限保持)。BoneNames と同じ添字で引ける。
        public readonly List<string> BoneNames = new();
        public readonly List<Vec3> BonePositions = new();
        public readonly List<int> BoneParents = new();       // 親ボーンindex (-1 = ルート)
        public readonly List<int> BoneDeformLayers = new();  // 変形階層 (FK計算順の参考)
    }
}
