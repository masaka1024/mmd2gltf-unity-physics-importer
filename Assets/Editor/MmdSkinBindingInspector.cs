using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace Mmd2GltfImporter
{
    /// <summary>
    /// スキン結線検査：SkinnedMeshRendererが参照しているボーン(bones配列＋ウェイト)と、
    /// 揺れ物ボーンが同一かを機械的に突き合わせる。
    /// （揺れ物の定義は GLB の extras.mmd 由来。MmdEditorShared.TryCollectSwayBones を参照）
    ///
    /// 背景：物理(剛体)は動き、クリップにも揺れ物カーブが無いのに絵が動かない場合、
    /// 「メッシュが物理の乗っているボーンにスキニングされていない」断線が疑われる。
    /// glTFインポートでのスケルトン二重化や、物理リグが別階層に組まれた場合に起こる。
    ///
    /// 使い方：MMD Physics → スキン結線検査 を開き、Scene上のモデルを指定して「検査」。
    /// </summary>
    public class MmdSkinBindingInspector : EditorWindow
    {
        [MenuItem("MMD Physics/スキン結線検査", false, 21)]
        public static void ShowWindow()
        {
            GetWindow<MmdSkinBindingInspector>("スキン結線検査");
        }

        [SerializeField] private GameObject modelRoot;
        [SerializeField] private Vector2 scroll;
        [SerializeField] private string lastReport = "";

        private void OnGUI()
        {
            EditorGUILayout.HelpBox(
                "SkinnedMeshRendererの参照ボーンと、揺れ物ボーンが同一インスタンスかを\n" +
                "検査します。別物なら、物理がいくら動いてもメッシュは動きません\n" +
                "（＝数字だけ動いて絵が鉄壁になる断線）。\n" +
                "揺れ物は GLB の extras.mmd（物理演算剛体 mode≠0）から特定します。",
                MessageType.Info);

            modelRoot = (GameObject)EditorGUILayout.ObjectField("モデル (Scene上)", modelRoot, typeof(GameObject), true);

            using (new EditorGUI.DisabledScope(modelRoot == null))
            {
                if (GUILayout.Button("検査", GUILayout.Height(28))) Inspect();
            }

            if (!string.IsNullOrEmpty(lastReport))
            {
                scroll = EditorGUILayout.BeginScrollView(scroll);
                EditorGUILayout.TextArea(lastReport, GUILayout.ExpandHeight(true));
                EditorGUILayout.EndScrollView();
            }
        }

        private static string LabelOf(string name) => MmdEditorShared.ClassifyPart(name);

        private void Inspect()
        {
            // 1) 揺れ物ボーンを GLB の extras.mmd から収集（定義は MmdEditorShared 参照）。
            //    ここが 0 件のまま先へ進むと「断線なし」と誤診するので、理由を出して止める。
            if (!MmdEditorShared.TryCollectSwayBones(modelRoot, out var swayMap, out string collectError))
            {
                lastReport = "★検査できません: 揺れ物ボーンを特定できませんでした。\n  " + collectError;
                Debug.LogWarning("[スキン結線検査] " + collectError);
                return;
            }
            var swayBones = new List<Transform>(swayMap.Keys);

            // 2) 全SkinnedMeshRendererから「参照されているボーン→合計ウェイト」を作る
            //    （bones配列に居ても実ウェイト0なら絵は動かないため、両方を見る）
            var refWeight = new Dictionary<Transform, float>();
            var refOnly = new HashSet<Transform>();
            var smrs = modelRoot.GetComponentsInChildren<SkinnedMeshRenderer>(true);
            foreach (var smr in smrs)
            {
                var bones = smr.bones;
                foreach (var b in bones) if (b != null) refOnly.Add(b);

                var mesh = smr.sharedMesh;
                if (mesh == null) continue;

                // ウェイト集計（レガシー4本API。空ならGetAllBoneWeightsで代替）
                var bw = mesh.boneWeights;
                if (bw != null && bw.Length > 0)
                {
                    foreach (var w in bw)
                    {
                        Accum(refWeight, bones, w.boneIndex0, w.weight0);
                        Accum(refWeight, bones, w.boneIndex1, w.weight1);
                        Accum(refWeight, bones, w.boneIndex2, w.weight2);
                        Accum(refWeight, bones, w.boneIndex3, w.weight3);
                    }
                }
                else
                {
                    var all = mesh.GetAllBoneWeights();
                    for (int i = 0; i < all.Length; i++)
                    {
                        var w = all[i];
                        Accum(refWeight, bones, w.boneIndex, w.weight);
                    }
                }
            }

            // 3) 突き合わせ
            var byLabel = new Dictionary<string, int[]>(); // label → [総数, bones参照, 実ウェイトあり]
            var orphanSamples = new List<string>();
            foreach (var t in swayBones)
            {
                string label = LabelOf(t.name);
                if (!byLabel.ContainsKey(label)) byLabel[label] = new int[3];
                byLabel[label][0]++;
                bool referenced = refOnly.Contains(t);
                bool weighted = refWeight.TryGetValue(t, out float wsum) && wsum > 1e-3f;
                if (referenced) byLabel[label][1]++;
                if (weighted) byLabel[label][2]++;
                if (!weighted && orphanSamples.Count < 12)
                    orphanSamples.Add($"    {AnimationUtility.CalculateTransformPath(t, modelRoot.transform)}" +
                                      (referenced ? "（bonesには居るがウェイト0）" : "（bones配列に不在）"));
            }

            // 4) レポート
            var sb = new StringBuilder();
            sb.AppendLine($"SkinnedMeshRenderer: {smrs.Length} 個 ／ 揺れ物ボーン: {swayBones.Count} 本");
            foreach (var kv in byLabel.OrderBy(k => k.Key))
                sb.AppendLine($"  {kv.Key}: {kv.Value[0]}本中 bones参照 {kv.Value[1]}本 ／ 実ウェイトあり {kv.Value[2]}本");
            sb.AppendLine();

            bool anyOrphan = byLabel.Any(kv => kv.Value[2] < kv.Value[0]);
            if (!anyOrphan)
            {
                sb.AppendLine("★判定: 全ての揺れ物ボーンが実ウェイト付きでスキンに結線されています。断線なし。");
                sb.AppendLine("  → 犯人はさらに別。Play中Animator OFFテストと、手動でボーンを回す実験の結果を突き合わせましょう。");
            }
            else
            {
                sb.AppendLine("★判定: 実ウェイトを持たない揺れ物ボーンがあります——断線の疑いが濃厚です。");
                sb.AppendLine("  物理はこれらのボーンを動かしていますが、メッシュは聞いていません。");
                sb.AppendLine("  該当サンプル（最大12件）:");
                foreach (var s in orphanSamples) sb.AppendLine(s);
                sb.AppendLine();
                sb.AppendLine("  → 対処はスキンの結線先を剛体ボーンに繋ぎ直す（bones配列の差し替え）方向になります。");
                sb.AppendLine("     結果を共有してもらえれば、繋ぎ直しツールを組みます。");
            }
            lastReport = sb.ToString();
            Debug.Log("[スキン結線検査]\n" + lastReport);
        }

        private static void Accum(Dictionary<Transform, float> map, Transform[] bones, int idx, float w)
        {
            if (w <= 0f || idx < 0 || bones == null || idx >= bones.Length) return;
            var t = bones[idx];
            if (t == null) return;
            map.TryGetValue(t, out float cur);
            map[t] = cur + w;
        }
    }
}
