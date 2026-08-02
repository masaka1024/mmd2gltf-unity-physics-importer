using System.Collections.Generic;
using UnityEngine;

namespace Mmd2GltfImporter
{
    /// <summary>
    /// 再生開始直後だけ揺れ物を Kinematic に固定し、少し経ってから物理へ引き渡す。
    ///
    /// バインドポーズとモーション1フレーム目の姿勢が違う場合、再生した瞬間に
    /// 体のボーンだけが新しい姿勢へ飛ぶ。揺れ物は動的なのでその場に取り残され、
    /// ジョイントは1ステップで巨大な誤差を突きつけられる。慣性の小さい髪では
    /// それが数千 rad/s の角速度に化け、収まるまで十秒以上暴れ続ける。
    ///
    /// 固定している間、揺れ物はボーンに追従して運ばれるだけなので、
    /// 解放時の拘束誤差はゼロになる。
    /// </summary>
    [DisallowMultipleComponent]
    public class MmdPhysicsWarmup : MonoBehaviour
    {
        [Tooltip("物理を止めておく秒数。アニメーションが最初の姿勢を確定させるのに必要な時間。")]
        public float warmupSeconds = 0.2f;

        [Tooltip("解放時に速度をゼロにする。運ばれている間についた見かけの速度を持ち込まない。")]
        public bool clearVelocityOnRelease = true;

        public bool logOnRelease = true;

        private readonly List<Rigidbody> held = new List<Rigidbody>();
        private float releaseTime;
        private bool released;

        void Awake()
        {
            held.Clear();
            foreach (var rb in GetComponentsInChildren<Rigidbody>())
            {
                if (rb == null || rb.isKinematic) continue;
                rb.isKinematic = true;
                held.Add(rb);
            }
            released = held.Count == 0;
            releaseTime = -1f;
        }

        void FixedUpdate()
        {
            if (released) return;

            // 最初の FixedUpdate を基準にする（Time.time の初期値に依存しないため）
            if (releaseTime < 0f)
            {
                releaseTime = Time.time + Mathf.Max(0f, warmupSeconds);
                return;
            }

            if (Time.time < releaseTime) return;

            foreach (var rb in held)
            {
                if (rb == null) continue;
                rb.isKinematic = false;
                if (clearVelocityOnRelease)
                {
                    rb.linearVelocity = Vector3.zero;
                    rb.angularVelocity = Vector3.zero;
                }
            }

            released = true;
            if (logOnRelease)
                Debug.Log($"[MMD 始動] {held.Count} 個の揺れ物を物理へ引き渡しました（t={Time.time:F2}s）。" +
                          "ここまではボーンに追従していたため、拘束誤差ゼロから始まります。");
        }
    }
}
