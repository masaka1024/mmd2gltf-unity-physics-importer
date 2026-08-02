using System.Collections.Generic;
using UnityEngine;

namespace Mmd2GltfImporter
{
    /// <summary>
    /// 揺れ物にかかる重力を MMD の単位系に合わせる。
    ///
    /// MMD の物理は PMX の単位系（1単位 ≒ 0.08m 相当）で動く。ここで重要なのは、
    /// 本家(VMDビューア)が Bullet へ渡している重力が 9.81 そのものではなく
    /// **9.81 × GravityBaseScale(既定10) = 98.1**（MMD単位/s²）だという点。
    /// MMDモデルは実寸の約10倍スケール（身長≈20単位）なので、この ×10 で
    /// 見た目の落下速度を実寸相当に合わせている。
    ///
    /// 実メートルに直すと 98.1 × 0.08 = 7.85 m/s² ＝ 約 0.8G。
    /// したがって Unity 側の倍率は unitScale × 10 = 0.8 が正しい。
    ///
    /// 単純に unitScale 倍（0.08）にすると重力が10分の1になり、
    /// 髪もスカートも垂れず「硬い」「曲がりが足りない」症状になる。
    /// </summary>
    [DisallowMultipleComponent]
    public class MmdGravity : MonoBehaviour
    {
        [Tooltip("重力の倍率。1.0 = Unity のまま。\n" +
                 "MMDと揃えるなら unitScale × GravityBaseScale = 0.08 × 10 = 0.8。")]
        public float gravityScale = 0.8f;

        public bool logOnApply = true;

        private readonly List<Rigidbody> bodies = new List<Rigidbody>();

        void Awake()
        {
            bodies.Clear();
            foreach (var rb in GetComponentsInChildren<Rigidbody>())
            {
                if (rb == null) continue;
                bodies.Add(rb);
                rb.useGravity = false; // 自前でかけるので標準の重力は切る
            }

            if (logOnApply)
                Debug.Log($"[MMD 重力] {bodies.Count} 個の剛体に倍率 {gravityScale:F3} の重力を適用します" +
                          $"（{Physics.gravity.magnitude * gravityScale:F3} m/s²）。" +
                          (Mathf.Approximately(gravityScale, 1f) ? "" : "揺れの周期が本家と揃います。"));
        }

        void FixedUpdate()
        {
            Vector3 g = Physics.gravity * gravityScale;
            for (int i = 0; i < bodies.Count; i++)
            {
                var rb = bodies[i];
                if (rb == null || rb.isKinematic) continue;
                rb.AddForce(g, ForceMode.Acceleration);
            }
        }
    }
}
