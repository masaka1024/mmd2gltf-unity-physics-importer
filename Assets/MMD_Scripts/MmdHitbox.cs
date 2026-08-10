using UnityEngine;

namespace Mmd2GltfImporter
{
    /// <summary>
    /// ゲーム内の当たり判定用ヒットボックスの目印。
    ///
    /// PMX の剛体定義（extras.mmd.rigidBodies）から作ったコライダーに1つずつ付く。
    /// 物理計算には一切関与しない（Rigidbody は付けず、コライダーはボーンの子として
    /// 追従するだけ）。揺れ物の物理は自作Bulletエンジン MmdPhysicsBehaviour が担当する。
    ///
    /// 用途は2つ:
    ///   ① 当たった部位を名前で知る。
    ///        void OnTriggerEnter(Collider other) {
    ///            var hit = other.GetComponent&lt;MmdHitbox&gt;();
    ///            if (hit != null) damage *= hit.PartName == "頭" ? 2f : 1f;
    ///        }
    ///   ② 生成し直し・除去のときに、インポーターが作ったものだけを走査する目印。
    ///
    /// ※Editorフォルダの外に置くこと（中に置くと実行時missing参照になる）。
    /// </summary>
    [DisallowMultipleComponent]
    public class MmdHitbox : MonoBehaviour
    {
        [Tooltip("PMX剛体の名前（例: 頭 / 上半身2 / 右ひざ）。当たった部位の判別に使う")]
        public string PartName;

        [Tooltip("追従しているボーンの名前")]
        public string BoneName;

        [Tooltip("PMXの物理モード 0=ボーン追従(体パーツ) / 1=物理演算(髪・スカート) / 2=物理+位置合わせ")]
        public int PhysicsMode;

        [Tooltip("PMXの衝突グループ (0〜15)")]
        public int Group;

        [Tooltip("extras.mmd.rigidBodies での通し番号")]
        public int RigidBodyIndex = -1;
    }
}
