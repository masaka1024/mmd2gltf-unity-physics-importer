using System.Collections.Generic;
using UnityEngine;

namespace Mmd2GltfImporter
{
    /// <summary>
    /// 揺れ物の動きを本家（PMXベイク）と同じ物差しで計測する。
    ///
    /// 見た目の「ところてん感」は2つの数字に分解できる。
    ///   ① 曲がり量 … ボーンが初期姿勢からどれだけ傾いているか（＝垂れ下がり）
    ///   ② 遅れ     … 親（頭・下半身）の動きにどれだけ遅れて追従するか
    ///
    /// IA + IA_Conqueror の本家ベイク実測値（目標）:
    ///   後ろ髪   曲がり 中央値 28.3° / p90 53.4°  遅れ 100ms
    ///   モミアゲ 曲がり 中央値 28.1° / p90 52.5°  遅れ 100ms
    ///   スカート 曲がり 中央値 12.7° / p90 26.6°  遅れ 200ms
    ///   前髪     曲がり 中央値  7.6° / p90 16.6°  遅れ  33ms
    ///
    /// 「戻ろうとする速さ(Hz)」を下げるほど曲がり量が増え、
    /// 「減衰比ζ」を上げるほど遅れが伸びる。数字を見ながら追い込める。
    /// </summary>
    [DisallowMultipleComponent]
    public class MmdMotionStats : MonoBehaviour
    {
        [Header("計測")]
        [Tooltip("この秒数だけ記録してから結果を出す。0 で手動（コンテキストメニュー）のみ。")]
        public float measureSeconds = 20f;

        [Tooltip("最初のこの秒数は捨てる（始動直後の助走を除くため）。")]
        public float skipSeconds = 1f;

        [Tooltip("遅れを測る最大の範囲（秒）。")]
        public float maxLagSeconds = 1.0f;

        [Header("内訳")]
        [Tooltip("この名前で始まるボーンを1本ずつ出す。チェーンのどこで遅れが積み上がるかが分かる。")]
        public string detailPrefix = "髪BC";

        [Header("参照ボーン（空なら名前で自動検出）")]
        public Transform headBone;
        public Transform lowerBodyBone;

        private class Track
        {
            public Transform t;
            public Quaternion rest;
            public string group;
            public List<float> angle = new List<float>();
        }

        private readonly List<Track> tracks = new List<Track>();
        private readonly List<float> headSpeed = new List<float>();
        private readonly List<float> lowSpeed = new List<float>();
        private Quaternion prevHead, prevLow;
        private float started = -1f;
        private bool done;

        void Start()
        {
            if (headBone == null) headBone = FindByName("頭");
            if (lowerBodyBone == null) lowerBodyBone = FindByName("下半身");

            // ★isKinematic で絞ってはいけない。
            //   MmdPhysicsWarmup が Awake で揺れ物を一時的に Kinematic にするため、
            //   Start の時点では全部 Kinematic に見えて1本も拾えない。
            //   揺れ物かどうかは名前（部位）で判断する。
            foreach (var rb in GetComponentsInChildren<Rigidbody>())
            {
                if (rb == null) continue;
                string g = GroupOf(rb.name);
                if (g == null) continue;
                tracks.Add(new Track { t = rb.transform, rest = rb.transform.localRotation, group = g });
            }

            if (tracks.Count == 0)
                Debug.LogWarning("[MMD 計測] 揺れ物が見つかりませんでした。" +
                                 "剛体がボーンに付いているか、ボーン名が 髪/前髪/モミアゲ/スカート で始まっているか確認してください。");

            if (headBone != null) prevHead = headBone.rotation;
            if (lowerBodyBone != null) prevLow = lowerBodyBone.rotation;

            Debug.Log($"[MMD 計測] {tracks.Count} 本の揺れ物を記録します" +
                      (measureSeconds > 0 ? $"（{measureSeconds:F0} 秒後に結果を出します）" : "（手動集計）") + "。");
        }

        private Transform FindByName(string n)
        {
            foreach (var t in GetComponentsInChildren<Transform>())
                if (t.name == n) return t;
            return null;
        }

        private static string GroupOf(string n)
        {
            if (n.StartsWith("前髪")) return "前髪";
            if (n.StartsWith("モミアゲ")) return "モミアゲ";
            if (n.StartsWith("髪")) return "髪(後)";
            if (n.StartsWith("スカート") && !n.Contains("横")) return "スカート";
            if (n.Contains("ツインテ")) return "髪(後)";
            return null;
        }

        void FixedUpdate()
        {
            if (done) return;
            if (started < 0f) { started = Time.time; return; }
            float el = Time.time - started;
            if (el < skipSeconds) return;

            foreach (var tr in tracks)
            {
                if (tr.t == null) { tr.angle.Add(0f); continue; }
                tr.angle.Add(Quaternion.Angle(tr.rest, tr.t.localRotation));
            }
            if (headBone != null)
            {
                headSpeed.Add(Quaternion.Angle(prevHead, headBone.rotation) / Time.fixedDeltaTime);
                prevHead = headBone.rotation;
            }
            if (lowerBodyBone != null)
            {
                lowSpeed.Add(Quaternion.Angle(prevLow, lowerBodyBone.rotation) / Time.fixedDeltaTime);
                prevLow = lowerBodyBone.rotation;
            }

            if (measureSeconds > 0f && el >= skipSeconds + measureSeconds)
            {
                Report();
                done = true;
            }
        }

        [ContextMenu("いま集計する")]
        public void Report()
        {
            if (tracks.Count == 0)
            {
                Debug.LogWarning("[MMD 計測] 揺れ物を1本も捕捉できていません（対象ボーンが見つかりません）。");
                return;
            }
            if (tracks[0].angle.Count < 10)
            {
                Debug.LogWarning($"[MMD 計測] 記録が {tracks[0].angle.Count} ステップしかありません。" +
                                 $"「捨てる秒数」({skipSeconds:F1}s)より長く再生してください。");
                return;
            }

            int maxLag = Mathf.Max(1, Mathf.RoundToInt(maxLagSeconds / Time.fixedDeltaTime));
            var byGroup = new Dictionary<string, List<Track>>();
            foreach (var tr in tracks)
            {
                List<Track> l;
                if (!byGroup.TryGetValue(tr.group, out l)) { l = new List<Track>(); byGroup[tr.group] = l; }
                l.Add(tr);
            }

            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"[MMD 計測] {tracks[0].angle.Count} ステップ分の結果（本家ベイクの実測値と並べています）");
            sb.AppendLine($"{"部位",-10}{"本数",4}{"曲がり中央",10}{"p90",8}{"遅れ",8}   本家の目標");

            foreach (var kv in byGroup)
            {
                var all = new List<float>();
                var lags = new List<float>();
                var refSpeed = kv.Key == "スカート" ? lowSpeed : headSpeed;

                foreach (var tr in kv.Value)
                {
                    all.AddRange(tr.angle);
                    float lg = BestLag(refSpeed, tr.angle, maxLag);
                    if (lg >= 0f) lags.Add(lg * Time.fixedDeltaTime * 1000f);
                }

                all.Sort();
                float med = all[all.Count / 2];
                float p90 = all[Mathf.Min(all.Count - 1, (int)(all.Count * 0.9f))];
                lags.Sort();
                float lagMed = lags.Count > 0 ? lags[lags.Count / 2] : -1f;

                string target =
                    kv.Key == "髪(後)" ? "28.3° / 53.4° / 100ms" :
                    kv.Key == "モミアゲ" ? "28.1° / 52.5° / 100ms" :
                    kv.Key == "スカート" ? "12.7° / 26.6° / 200ms" :
                    kv.Key == "前髪" ? " 7.6° / 16.6° /  33ms" : "";

                sb.AppendLine($"{kv.Key,-10}{kv.Value.Count,4}{med,9:F1}°{p90,7:F1}°{lagMed,6:F0}ms   {target}");
            }

            sb.AppendLine("   ※曲がりが足りない＝硬い → 「戻ろうとする速さ(Hz)」を下げる");
            sb.AppendLine("   ※遅れが足りない → 「減衰比ζ」を上げる");

            // ── チェーンの内訳（根元から先端へ、遅れがどう積み上がるか）──
            if (!string.IsNullOrEmpty(detailPrefix))
            {
                var chain = new List<Track>();
                foreach (var tr in tracks)
                    if (tr.t != null && tr.t.name.StartsWith(detailPrefix)) chain.Add(tr);
                chain.Sort((a2, b2) => string.CompareOrdinal(a2.t.name, b2.t.name));

                if (chain.Count > 0)
                {
                    sb.AppendLine();
                    sb.AppendLine($"■ '{detailPrefix}' の内訳（根元→先端）");
                    sb.AppendLine($"{"ボーン",-14}{"曲がり中央",10}{"p90",8}{"遅れ",8}");
                    var refSpeed2 = detailPrefix.StartsWith("スカート") ? lowSpeed : headSpeed;
                    foreach (var tr in chain)
                    {
                        var v = new List<float>(tr.angle); v.Sort();
                        float m2 = v[v.Count / 2];
                        float p2 = v[Mathf.Min(v.Count - 1, (int)(v.Count * 0.9f))];
                        float lg = BestLag(refSpeed2, tr.angle, maxLag);
                        string lagTxt = lg >= 0f ? $"{lg * Time.fixedDeltaTime * 1000f,6:F0}ms" : "     --";
                        sb.AppendLine($"{tr.t.name,-14}{m2,9:F1}°{p2,7:F1}°{lagTxt}");
                    }
                    sb.AppendLine("   （本家の髪BC: 100 → 100 → 167 → 267ms と先端へ向かって積み上がる）");
                }
            }

            Debug.Log(sb.ToString());
        }

        // 参照の角速度と、揺れ物の曲がり量の相互相関がいちばん高くなるずれ量を返す
        private static float BestLag(List<float> refSig, List<float> sig, int maxLag)
        {
            int n = Mathf.Min(refSig.Count, sig.Count);
            if (n < maxLag + 10) return -1f;

            float rm = 0f, sm = 0f;
            for (int i = 0; i < n; i++) { rm += refSig[i]; sm += sig[i]; }
            rm /= n; sm /= n;

            float best = -2f; int bl = -1;
            for (int L = 0; L <= maxLag; L++)
            {
                double num = 0, da = 0, db = 0;
                for (int i = 0; i + L < n; i++)
                {
                    double a = refSig[i] - rm, b = sig[i + L] - sm;
                    num += a * b; da += a * a; db += b * b;
                }
                if (da < 1e-9 || db < 1e-9) continue;
                float c = (float)(num / System.Math.Sqrt(da * db));
                if (c > best) { best = c; bl = L; }
            }
            return best > 0.1f ? bl : -1f;
        }
    }
}