using System.Collections.Generic;
using UnityEngine;

namespace Mmd2GltfImporter
{
    /// <summary>
    /// ★Bullet真似モード：本家MMD(Bullet)とPhysXのエンジン差を、対象剛体に限定して模倣する。
    ///
    /// 背景（2026-08-01〜02の調査で特定したエンジン差）：
    ///  1. Bulletは少ない反復で拘束をゆるく解くため、強い負荷で拘束が「破れる」。
    ///     本家の翻り(ターン時57°超)の超過分はこのカオス由来。→ solverIterationsを下げて模倣
    ///  2. Bulletはめり込みを衝撃的に弾き出す。→ maxDepenetrationVelocityの上限解放で模倣
    ///  3. Bulletの減衰d=毎秒(1-d)倍（0.9なら毎秒90%減）。Unityは同じ数字でも弱い。
    ///     → D=-ln(1-d)の忠実変換（※翻りは減るかもしれない。忠実さのトグル）
    ///
    /// 使い方：モデルのルートにアタッチしてPlay。1トグルずつA/Bし、効果は
    /// MmdJointProbeのターンイベント(★傾き)と目視で判定する。
    /// </summary>
    public class MmdBulletMimicry : MonoBehaviour
    {
        [Tooltip("対象剛体の名前フィルタ（この文字列を含む非Kinematic剛体だけに適用）")]
        public string nameFilter = "スカート";

        [Header("① 拘束を破れるようにする（本命・未試験レバー）")]
        [Tooltip("ONで対象剛体のソルバ反復数を下げ、Bulletのように拘束が負荷で破れる状態にする")]
        public bool lowerSolverIterations = true;
        [Tooltip("位置ソルバの反復数（PhysX既定6。1〜2でBullet的なゆるさ。下げるほど破れやすいが暴れやすい）")]
        public int solverIterations = 2;
        [Tooltip("速度ソルバの反復数（PhysX既定1のプロジェクトが多い。据え置きなら1）")]
        public int solverVelocityIterations = 1;

        [Header("② めり込みの弾き出しを解放")]
        [Tooltip("ONで対象剛体のmaxDepenetrationVelocityを引き上げ、Bulletの衝撃的な弾き出しを模倣")]
        public bool uncapDepenetration = false;
        [Tooltip("解放時の上限 [m/s]（現行は全体で1に制限中）")]
        public float depenetrationVelocity = 10f;

        [Header("③ 減衰の忠実変換（翻りは減る可能性あり・忠実さ優先トグル）")]
        [Tooltip("ONで減衰をBullet意味論へ忠実変換：D = -ln(1-d)。例 0.9→約2.3（今より強い減衰になる）")]
        public bool convertDampingToBullet = false;

        private class Original
        {
            public Rigidbody rb;
            public int si, svi;
            public float depen, linD, angD;
        }
        private readonly List<Original> _originals = new List<Original>();
        private bool _captured;

        private void Awake()
        {
            Apply();
        }

        [ContextMenu("再適用")]
        public void Apply()
        {
            if (!_captured)
            {
                foreach (var rb in GetComponentsInChildren<Rigidbody>())
                {
                    if (rb.isKinematic) continue;
                    if (!string.IsNullOrEmpty(nameFilter) && !rb.name.Contains(nameFilter)) continue;
                    _originals.Add(new Original
                    {
                        rb = rb,
                        si = rb.solverIterations,
                        svi = rb.solverVelocityIterations,
                        depen = rb.maxDepenetrationVelocity,
                        linD = rb.linearDamping,
                        angD = rb.angularDamping,
                    });
                }
                _captured = true;
            }

            int n = 0;
            foreach (var o in _originals)
            {
                if (o.rb == null) continue;
                n++;

                if (lowerSolverIterations)
                {
                    o.rb.solverIterations = Mathf.Max(1, solverIterations);
                    o.rb.solverVelocityIterations = Mathf.Max(1, solverVelocityIterations);
                }
                else
                {
                    o.rb.solverIterations = o.si;
                    o.rb.solverVelocityIterations = o.svi;
                }

                o.rb.maxDepenetrationVelocity = uncapDepenetration ? depenetrationVelocity : o.depen;

                if (convertDampingToBullet)
                {
                    // Bullet: v *= (1-d) per second → 等価なUnity damping D = -ln(1-d)
                    o.rb.linearDamping = -Mathf.Log(Mathf.Max(1e-3f, 1f - Mathf.Clamp(o.linD, 0f, 0.98f)));
                    o.rb.angularDamping = -Mathf.Log(Mathf.Max(1e-3f, 1f - Mathf.Clamp(o.angD, 0f, 0.98f)));
                }
                else
                {
                    o.rb.linearDamping = o.linD;
                    o.rb.angularDamping = o.angD;
                }
            }

            Debug.Log($"[Bullet真似モード] 「{nameFilter}」剛体 {n} 本へ適用: " +
                      $"①反復低下={(lowerSolverIterations ? $"ON(pos{solverIterations}/vel{solverVelocityIterations})" : "OFF")} " +
                      $"②弾き出し解放={(uncapDepenetration ? $"ON({depenetrationVelocity:F0}m/s)" : "OFF")} " +
                      $"③減衰忠実変換={(convertDampingToBullet ? "ON" : "OFF")}");
        }

        [ContextMenu("元に戻す")]
        public void RestoreAll()
        {
            foreach (var o in _originals)
            {
                if (o.rb == null) continue;
                o.rb.solverIterations = o.si;
                o.rb.solverVelocityIterations = o.svi;
                o.rb.maxDepenetrationVelocity = o.depen;
                o.rb.linearDamping = o.linD;
                o.rb.angularDamping = o.angD;
            }
            Debug.Log("[Bullet真似モード] 全て元の値に戻しました");
        }
    }
}
