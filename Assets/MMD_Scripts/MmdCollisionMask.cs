using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace Mmd2GltfImporter
{
    /// <summary>
    /// PMX の衝突グループ／マスクを Physics.IgnoreCollision でペア単位に再現する。
    ///
    /// 【マスクの意味】立っているビットは「そのグループと衝突する」を意味する
    /// （Bullet の collision filter mask と同じ。PMXエディタのチェック表示＝
    ///  非衝突とは反転している）。衝突条件は双方向で、
    ///  (1&lt;&lt;A.group &amp; B.mask) &amp;&amp; (1&lt;&lt;B.group &amp; A.mask) のときだけ衝突する。
    ///
    /// レイヤーによる一括指定では表現できない「この剛体とこの剛体だけ当たらない」を
    /// 作者の指定どおりに再現する。とくに髪の根元やスカートの中段は体のコライダーの
    /// 内側に埋まっているのが普通で、作者はそれを前提に静止姿勢を作っている。
    /// レイヤー方式ではその指定が失われ、毎フレーム押し出され続けて発散する。
    ///
    /// Physics.IgnoreCollision はシーンに保存されないため、再生のたびに
    /// このコンポーネントが Awake で貼り直す。
    /// </summary>
    [DisallowMultipleComponent]
    public class MmdCollisionMask : MonoBehaviour
    {
        [SerializeField] private List<Collider> colliders = new List<Collider>();
        [SerializeField] private List<int> groups = new List<int>();
        [SerializeField] private List<int> masks = new List<int>();
        [SerializeField] private bool logOnApply = true;

        [Tooltip("静止姿勢ですでに食い込んでいる組を、いったん衝突対象から外す。\n" +
                 "MMDのモデルは揺れ物の根元が体のコライダーに埋まっているのが普通で、\n" +
                 "作者はそれを前提に静止姿勢を作っている。押し出そうとすると再生と同時に外へ弾かれる。")]
        public bool ignoreRestEmbeddedPairs = true;

        [Tooltip("いったん外した組を、埋まりが解消した時点で本来の衝突に戻す。\n" +
                 "腰のように構造上ずっと埋まったままの組は外れたまま、\n" +
                 "脚を上げて離れた組はその瞬間から当たるようになるため、すり抜けを防げる。")]
        public bool restoreWhenSeparated = true;

        [Header("診断")]
        [Tooltip("保留にした組の全リスト（ペア名＋食い込み深さ）をApply時にログへ出す。\n" +
                 "「脚がスカートを押せない状態になっていないか」等の切り分け用。")]
        public bool dumpPendingPairs = true;

        [Tooltip("保留サマリで件数を数える名前フィルタ。\n" +
                 "保留のうちこの文字列を含む組が何組あるかを集計する。")]
        public string tallyNameFilter = "スカート";

        [Header("A/Bテスト")]
        [Tooltip("この文字列を名前に含むコライダーが絡む組は、静止姿勢で食い込んでいても\n" +
                 "保留にせず最初から衝突を有効化する（例:「スカート」）。\n" +
                 "空欄=無効。\n" +
                 "注意: 埋まり保留の安全機構を一部外すため、再生直後にスカートが\n" +
                 "外へ押し出される・弾かれる等の副作用が出る可能性がある。切り分け用。")]
        public string forceActiveNameFilter = "";

        // 埋まりが解消するのを待っている組（colliders のインデックス対）
        private readonly List<int> pendingA = new List<int>();
        private readonly List<int> pendingB = new List<int>();
        private int releasedTotal;
        private float nextLogTime;

        public void Setup(List<Collider> cols, List<int> grps, List<int> msks)
        {
            colliders = cols;
            groups = grps;
            masks = msks;
        }

        void Awake()
        {
            Apply();
        }

        public void Apply()
        {
            if (colliders == null || colliders.Count == 0) return;

            pendingA.Clear();
            pendingB.Clear();
            releasedTotal = 0;

            int n = colliders.Count;
            int ignored = 0, kept = 0, skipped = 0;
            float embeddedMax = 0f; string embeddedWorst = "";

            // ★診断用：保留ペアの詳細と、強制有効化した（保留にしなかった）ペアの記録
            var pendingDump = new StringBuilder();
            int pendingTally = 0;      // 保留のうち tallyNameFilter を含む組
            int forcedActive = 0;      // 食い込んでいたが forceActiveNameFilter により有効化した組
            float forcedMax = 0f; string forcedWorst = "";
            bool useForce = !string.IsNullOrEmpty(forceActiveNameFilter);

            for (int i = 0; i < n; i++)
            {
                var ci = colliders[i];
                if (ci == null) { skipped++; continue; }

                for (int j = i + 1; j < n; j++)
                {
                    var cj = colliders[j];
                    if (cj == null) continue;

                    bool collide = ((masks[j] & (1 << groups[i])) != 0)
                                && ((masks[i] & (1 << groups[j])) != 0);

                    // ★静止姿勢ですでに食い込んでいる組は、作者が意図して作った配置。
                    //   解消すべき侵入ではないので、いまは衝突させない。
                    //   （エクスポーター側で「構造上コライダー内に埋まっている親端の侵入を
                    //     子へ転写しない」とした対処と同じ考え方）
                    if (collide && ignoreRestEmbeddedPairs && Overlaps(ci, cj, out float dist))
                    {
                        // ★A/B: 指定フィルタに合致する組は保留にせず、食い込んだまま衝突を有効化
                        bool force = useForce && (NameMatches(ci, forceActiveNameFilter)
                                               || NameMatches(cj, forceActiveNameFilter));
                        if (force)
                        {
                            forcedActive++;
                            if (dist > forcedMax)
                            {
                                forcedMax = dist;
                                forcedWorst = $"{ci.name} ⇔ {cj.name}";
                            }
                            // collide は true のまま＝最初から当たる
                        }
                        else
                        {
                            collide = false;
                            pendingA.Add(i);
                            pendingB.Add(j);
                            if (dist > embeddedMax)
                            {
                                embeddedMax = dist;
                                embeddedWorst = $"{ci.name} ⇔ {cj.name}";
                            }
                            if (NameMatches(ci, tallyNameFilter) || NameMatches(cj, tallyNameFilter))
                                pendingTally++;
                            if (dumpPendingPairs)
                                pendingDump.AppendLine($"    {ci.name} ⇔ {cj.name}  食い込み {dist * 1000f:F1}mm");
                        }
                    }

                    Physics.IgnoreCollision(ci, cj, !collide);
                    if (collide) kept++; else ignored++;
                }
            }

            if (logOnApply)
            {
                Debug.Log($"[MMD 衝突マスク] {n} 個のコライダーに PMX のグループ／マスクを適用しました" +
                          $"（衝突する組 {kept} / 無効化した組 {ignored}"
                          + (skipped > 0 ? $" / 参照切れ {skipped}" : "") + "）。");
                if (pendingA.Count > 0)
                {
                    string tallyNote = string.IsNullOrEmpty(tallyNameFilter)
                        ? ""
                        : $" うち「{tallyNameFilter}」絡みは {pendingTally} 組。";
                    Debug.Log($"[MMD 衝突マスク] うち {pendingA.Count} 組は静止姿勢ですでに食い込んでいるため保留にしました" +
                              $"（最大 {embeddedMax:F4} m: {embeddedWorst}）。{tallyNote}" +
                              (restoreWhenSeparated ? "離れた時点で衝突を復活させます。" : "復活はしません。"));
                    if (dumpPendingPairs && pendingDump.Length > 0)
                        Debug.Log($"[MMD 衝突マスク] 保留 {pendingA.Count} 組の内訳:\n{pendingDump}");
                }
                if (forcedActive > 0)
                    Debug.Log($"[MMD 衝突マスク] ★A/B: 「{forceActiveNameFilter}」絡みの {forcedActive} 組は食い込みがあっても" +
                              $"保留にせず、最初から衝突を有効化しました（最大 {forcedMax:F4} m: {forcedWorst}）。");
            }
        }

        void FixedUpdate()
        {
            if (!restoreWhenSeparated || pendingA.Count == 0) return;

            int released = 0;

            // 後ろから走査して、解消した組をその場で取り除く
            for (int k = pendingA.Count - 1; k >= 0; k--)
            {
                var ca = colliders[pendingA[k]];
                var cb = colliders[pendingB[k]];
                if (ca == null || cb == null)
                {
                    pendingA.RemoveAt(k); pendingB.RemoveAt(k);
                    continue;
                }

                if (Overlaps(ca, cb, out _)) continue; // まだ埋まっている

                // 離れた ＝ 本来の衝突に戻す。以後は戻さない（戻すとすり抜けの元になる）
                Physics.IgnoreCollision(ca, cb, false);
                pendingA.RemoveAt(k); pendingB.RemoveAt(k);
                released++;
            }

            if (released > 0)
            {
                releasedTotal += released;
                if (logOnApply && Time.time >= nextLogTime)
                {
                    nextLogTime = Time.time + 1f;
                    Debug.Log($"[MMD 衝突マスク] 埋まりが解消した {releasedTotal} 組の衝突を復活させました" +
                              $"（保留の残り {pendingA.Count} 組）。");
                }
            }
        }

        // ★コライダー名または所属剛体名にフィルタ文字列を含むか
        private static bool NameMatches(Collider c, string filter)
        {
            if (string.IsNullOrEmpty(filter)) return false;
            if (c.name.Contains(filter)) return true;
            var rb = c.attachedRigidbody;
            return rb != null && rb.name.Contains(filter);
        }

        private static bool Overlaps(Collider a, Collider b, out float dist)
        {
            Vector3 dir;
            bool hit = Physics.ComputePenetration(
                a, a.transform.position, a.transform.rotation,
                b, b.transform.position, b.transform.rotation,
                out dir, out dist);
            return hit && dist > 1e-5f;
        }
    }
}