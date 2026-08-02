using UnityEngine;

namespace Mmd2GltfImporter
{
    /// <summary>
    /// ★ベイク再生モード（検証用）：ライブ物理を止め、Animator（ベイク済みクリップ）に
    /// 揺れ物ボーンを完全に委ねる。
    ///
    /// 用途：本家で物理ベイクしたフルキーVMD由来のクリップを「答え」としてこのモデルで再生し、
    /// スキン・メッシュ・描画を通した100%の見た目基準を得る。ライブ物理(75-80%再現)との
    /// 視覚的な差分確認や、期待値の較正に使う。
    ///
    /// 使い方：モデルのルートにアタッチしてPlay。ライブ物理に戻すときはコンポーネントを外すだけ。
    /// 注意：MmdSpinTestが付いているとAnimatorを無効化してしまうため、外すか無効にすること。
    /// </summary>
    public class MmdBakedPlaybackMode : MonoBehaviour
    {
        [Tooltip("Kinematic化を維持する監視時間[秒]。Warmup等が後から剛体を非Kinematicに戻すのを防ぐ")]
        public float enforceSeconds = 3f;

        private float _t;
        private int _initialCount;

        private void Awake()
        {
            // Warmup系（剛体を一時Kinematic化→後で解除するコンポーネント）は
            // 解除処理がこのモードと衝突するため、名前で見つけて無効化する
            foreach (var mb in GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (mb == null || mb == this) continue;
                string tn = mb.GetType().Name;
                if (tn == "MmdPhysicsWarmup" || tn == "MmdJointProbe")
                {
                    mb.enabled = false;
                    Debug.Log($"[ベイク再生モード] {tn} を無効化しました（ライブ物理系のため）");
                }
            }

            _initialCount = EnforceKinematic();
            Debug.Log($"[ベイク再生モード] 揺れ物剛体 {_initialCount} 本をKinematic化しました。" +
                      "Animatorのベイク済みクリップがボーンを直接駆動します（これが100%の見た目基準です）。");
        }

        private void Update()
        {
            // Warmupのコルーチン等が後から isKinematic=false に戻すケースへの保険
            _t += Time.deltaTime;
            if (_t > enforceSeconds) return;
            int fixedNow = EnforceKinematic();
            if (fixedNow > 0)
                Debug.Log($"[ベイク再生モード] 後から非Kinematic化された {fixedNow} 本を再Kinematic化しました");
        }

        private int EnforceKinematic()
        {
            int n = 0;
            foreach (var rb in GetComponentsInChildren<Rigidbody>())
            {
                if (rb.isKinematic) continue;
                rb.isKinematic = true;
                n++;
            }
            return n;
        }
    }
}
