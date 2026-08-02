using UnityEngine;

namespace Mmd2GltfImporter
{
    /// <summary>
    /// 接触の強さを記録するだけの観測用コンポーネント。
    /// 剛体の付いた GameObject に貼ると、そのフレームで受けた最大の衝撃と
    /// 相手の名前を保持する。MmdPhysicsWatcher が定期的に集計して出力する。
    /// </summary>
    [DisallowMultipleComponent]
    public class MmdContactProbe : MonoBehaviour
    {
        [System.NonSerialized] public float maxImpulse;
        [System.NonSerialized] public string maxPartner;
        [System.NonSerialized] public int contactCount;

        void OnCollisionStay(Collision c)
        {
            contactCount++;
            float imp = c.impulse.magnitude;
            if (imp > maxImpulse)
            {
                maxImpulse = imp;
                maxPartner = c.collider != null ? c.collider.name : "(不明)";
            }
        }

        void OnCollisionEnter(Collision c)
        {
            OnCollisionStay(c);
        }

        public void ResetStats()
        {
            maxImpulse = 0f;
            maxPartner = null;
            contactCount = 0;
        }
    }
}
