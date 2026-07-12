using UnityEngine;

namespace Mmd2GltfImporter
{
    // 剛体の並び順(rigidBodies配列の番号)を記録しておくための目印コンポーネント。
    // ※Editorフォルダの外に置くこと（中に置くと実行時missing参照になる）。
    public class MmdPhysicsImportIndex : MonoBehaviour
    {
        public int absoluteDataIndex = -1;
        public string boneName = "";

        // ★ジョイントの親剛体から見た「本来あるべき相対姿勢」（Warmupで使用）。
        //   ConnectJoints実行時（バインドポーズ）に記録し、Play開始直後の
        //   「助走」でこの姿勢へ追従させ、モーション開始の弾き飛びを防ぐ。
        public bool hasRestPose = false;
        public Vector3 restLocalPos = Vector3.zero;
        public Quaternion restLocalRot = Quaternion.identity;

        // ★物理の揺れを実際のメッシュ(ボーン)へ書き戻すための基準値。
        //   この剛体はボーンの子にせず中立の入れ物へ置く(親子関係にすると
        //   書き戻しがフィードバックループになり暴れるため)。そのため
        //   「ボーンへの直接参照」と「ワールド回転同士で計算した基準の相対回転」
        //   を持たせておき、毎フレーム同じ計算式で差分を求める。
        public bool hasBoneWriteback = false;
        public Transform boneTransformRef; // 書き戻し先の実ボーン（親子関係ではなく直接参照）
        public Quaternion restRbLocalRotToBone = Quaternion.identity; // ボーンから見た剛体の基準相対回転（ワールド回転同士で算出）
        public Quaternion restBoneLocalRotation = Quaternion.identity; // ボーン自身の基準ローカル回転
    }
}
