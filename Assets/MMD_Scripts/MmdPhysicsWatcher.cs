using System.Collections.Generic;
using UnityEngine;

namespace Mmd2GltfImporter
{
    /// <summary>
    /// 観測用コンポーネント。Play 中に MMD 剛体を監視し、
    /// 「最初に発散した1体」を名指しする。
    ///
    /// 静止状態で暴れる場合、連鎖の起点さえ分かれば原因はほぼ絞れる。
    /// 二番目以降は巻き添えなので、最初の1体だけが重要。
    /// </summary>
    [DisallowMultipleComponent]
    public class MmdPhysicsWatcher : MonoBehaviour
    {
        [Header("しきい値（超えたら発散とみなす）")]
        public float moveThreshold = 0.3f;    // 初期位置からの移動量（m）
        public float linVelThreshold = 10f;   // 速度（m/s）
        public float angVelThreshold = 30f;   // 角速度（rad/s）

        [Header("接触の記録")]
        [Tooltip("剛体ごとに接触の強さを記録し、1秒ごとに強い順で出力する。\n" +
                 "どの体のコライダーが揺れ物を弾いているのかを名指しできる。")]
        public bool probeContacts = true;

        [Header("動作")]
        public bool pauseOnFirst = true;      // 最初の検出でエディタを一時停止する
        public int maxReports = 5;
        public bool logPeakEverySecond = false; // 1秒ごとに最大速度の剛体を出す

        private Rigidbody[] bodies;
        private Vector3[] initialLocal;
        private Transform root;
        private int reports;
        private int fixedFrame;
        private bool armed;
        private float nextPeakLog;

        void Start()
        {
            root = transform;
            bodies = GetComponentsInChildren<Rigidbody>();
            // ※MmdPhysicsWarmup と併用する場合、この時点では揺れ物がまだ Kinematic なので
            //   「動的 0」と表示される。初期位置も助走前のものになるため、解放直後に
            //   移動量の誤検出が出ることがある（実害はない）。
            initialLocal = new Vector3[bodies.Length];
            for (int i = 0; i < bodies.Length; i++)
                initialLocal[i] = root.InverseTransformPoint(bodies[i].worldCenterOfMass);

            reports = 0;
            fixedFrame = 0;
            armed = true;
            nextPeakLog = 1f;

            if (probeContacts)
            {
                foreach (var rb in bodies)
                {
                    if (rb == null || rb.isKinematic) continue;
                    if (rb.GetComponent<MmdContactProbe>() == null)
                        rb.gameObject.AddComponent<MmdContactProbe>();
                }
            }

            int dyn = 0;
            foreach (var rb in bodies) if (rb != null && !rb.isKinematic) dyn++;

            Debug.Log($"[MMD Watch] 監視開始: 剛体 {bodies.Length} 体（うち動的 {dyn}） / " +
                      $"fixedDeltaTime={Time.fixedDeltaTime:F4} ({1f / Time.fixedDeltaTime:F0}Hz) / " +
                      $"Physics.defaultSolverIterations={Physics.defaultSolverIterations} / " +
                      $"defaultSolverVelocityIterations={Physics.defaultSolverVelocityIterations}");
        }

        void FixedUpdate()
        {
            if (bodies == null) return;
            fixedFrame++;

            float peak = 0f; Rigidbody peakRb = null;

            for (int i = 0; i < bodies.Length; i++)
            {
                var rb = bodies[i];
                if (rb == null || rb.isKinematic) continue;

                float lv = rb.linearVelocity.magnitude;
                if (lv > peak) { peak = lv; peakRb = rb; }

                if (!armed) continue;

                float move = Vector3.Distance(root.InverseTransformPoint(rb.worldCenterOfMass), initialLocal[i]);
                float av = rb.angularVelocity.magnitude;
                if (move < moveThreshold && lv < linVelThreshold && av < angVelThreshold) continue;

                Report(rb, move, lv, av);
                if (reports >= maxReports)
                {
                    armed = false;
                    Debug.Log("[MMD Watch] 報告上限に達したため監視を停止しました。");
                }
                if (pauseOnFirst)
                {
                    armed = false;
                    Debug.Break();
                }
            }

            if (Time.time >= nextPeakLog)
            {
                nextPeakLog = Time.time + 1f;
                if (logPeakEverySecond && peakRb != null)
                    Debug.Log($"[MMD Watch] t={Time.time:F1}s 最大速度 {peak:F2} m/s : {Path(peakRb.transform)}");
                if (probeContacts) ReportContacts();
            }
        }

        private void Report(Rigidbody rb, float move, float lv, float av)
        {
            reports++;

            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"[MMD Watch] ★発散 #{reports}（FixedUpdate {fixedFrame} 回目 / t={Time.time:F3}s）");
            sb.AppendLine($"  剛体: {Path(rb.transform)}");
            sb.AppendLine($"  初期位置からの移動 {move:F3} m / 速度 {lv:F2} m/s / 角速度 {av:F2} rad/s");
            sb.AppendLine($"  mass={rb.mass:F4} linearDamping={rb.linearDamping:F3} angularDamping={rb.angularDamping:F3}");
            sb.AppendLine($"  inertiaTensor={rb.inertiaTensor} (最小成分 {Mathf.Min(rb.inertiaTensor.x, Mathf.Min(rb.inertiaTensor.y, rb.inertiaTensor.z)):E2})");

            var joints = rb.GetComponents<ConfigurableJoint>();
            sb.AppendLine($"  この剛体に付くジョイント: {joints.Length} 個");
            foreach (var j in joints)
            {
                string parent = j.connectedBody != null ? Path(j.connectedBody.transform) : "(なし)";
                Vector3 wa = j.transform.TransformPoint(j.anchor);
                Vector3 wc = j.connectedBody != null
                    ? j.connectedBody.transform.TransformPoint(j.connectedAnchor)
                    : j.connectedAnchor;
                sb.AppendLine($"   ├ 親: {parent}");
                sb.AppendLine($"   ├ anchorズレ: {Vector3.Distance(wa, wc):F4} m（0に近いほど健全）");
                sb.AppendLine($"   ├ 直線: x={j.xMotion} y={j.yMotion} z={j.zMotion} limit={j.linearLimit.limit:F4}");
                sb.AppendLine($"   └ 回転: X[{j.lowAngularXLimit.limit:F1}..{j.highAngularXLimit.limit:F1}] " +
                              $"Y±{j.angularYLimit.limit:F1} Z±{j.angularZLimit.limit:F1} " +
                              $"（motion {j.angularXMotion}/{j.angularYMotion}/{j.angularZMotion}）");
            }

            Debug.LogWarning(sb.ToString(), rb.gameObject);
        }

        // 直近1秒で最も強い接触を受けた剛体を、相手の名前つきで上位5件出す。
        private void ReportContacts()
        {
            var probes = new List<MmdContactProbe>();
            foreach (var rb in bodies)
            {
                if (rb == null) continue;
                var p = rb.GetComponent<MmdContactProbe>();
                if (p != null && p.maxImpulse > 0f) probes.Add(p);
            }
            if (probes.Count == 0) return;

            probes.Sort((x, y) => y.maxImpulse.CompareTo(x.maxImpulse));

            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"[MMD Watch] t={Time.time:F1}s 強い接触の上位（この1秒間）:");
            int n = Mathf.Min(5, probes.Count);
            for (int i = 0; i < n; i++)
                sb.AppendLine($"   衝撃 {probes[i].maxImpulse:F3} : {probes[i].name} ← {probes[i].maxPartner}" +
                              $"（接触 {probes[i].contactCount} 回）");
            Debug.Log(sb.ToString());

            foreach (var p in probes) p.ResetStats();
        }

        private static string Path(Transform t)
        {
            var stack = new List<string>();
            while (t != null) { stack.Add(t.name); t = t.parent; }
            stack.Reverse();
            return string.Join("/", stack);
        }
    }
}