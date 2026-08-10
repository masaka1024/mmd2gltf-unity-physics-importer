using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace Mmd2GltfImporter
{
    /// <summary>
    /// クリップ診断：AnimationClipの中に「揺れ物ボーン」向けの
    /// カーブが入っていないかを検査する。
    /// （揺れ物の定義は GLB の extras.mmd 由来。MmdEditorShared.TryCollectSwayBones を参照）
    ///
    /// 背景：本体のみベイクのGLTFでも、パイプラインが全ノードに
    /// レストポーズの定数キー（静的チャンネル）を付けることがある。
    /// 定数カーブが1本でもあれば、Animatorが毎フレームそのボーンへ
    /// レストポーズを書き戻し、ライブ物理の結果を描画前に上書きしてしまう
    /// （＝スカートが体に固定された「鉄壁」の見た目になる）。
    ///
    /// 使い方：MMD Physics → クリップ診断 を開き、モデル(Scene上)とクリップを
    /// 指定して「検査」。揺れ物カーブが見つかったら「複製して揺れ物カーブを削除」で
    /// 除去済みクリップ(_stripped)を作り、Animatorに差し替える。
    /// </summary>
    public class MmdClipCurveInspector : EditorWindow
    {
        [MenuItem("MMD Physics/クリップ診断（揺れ物カーブ検査）", false, 20)]
        public static void ShowWindow()
        {
            GetWindow<MmdClipCurveInspector>("クリップ診断");
        }

        [SerializeField] private GameObject modelRoot;
        [SerializeField] private AnimationClip clip;
        [SerializeField] private Vector2 scroll;
        [SerializeField] private string lastReport = "";

        // 直近の検査で見つかった「揺れ物ボーンのパス」（削除ボタンで使う）
        private HashSet<string> _swayPathsInClip = new HashSet<string>();

        private void OnGUI()
        {
            EditorGUILayout.HelpBox(
                "クリップの中に、揺れ物ボーン（スカート・髪など）向けの\n" +
                "カーブが入っていないかを検査します。定数キーでもAnimatorは毎フレーム\n" +
                "書き戻すため、ライブ物理が描画前に上書きされます。\n" +
                "揺れ物は GLB の extras.mmd（物理演算剛体 mode≠0）から特定します。",
                MessageType.Info);

            modelRoot = (GameObject)EditorGUILayout.ObjectField("モデル (Scene上)", modelRoot, typeof(GameObject), true);
            clip = (AnimationClip)EditorGUILayout.ObjectField("AnimationClip", clip, typeof(AnimationClip), false);

            using (new EditorGUI.DisabledScope(modelRoot == null || clip == null))
            {
                if (GUILayout.Button("検査", GUILayout.Height(28))) Inspect();

                using (new EditorGUI.DisabledScope(_swayPathsInClip.Count == 0))
                {
                    if (GUILayout.Button($"複製して揺れ物カーブを削除（{_swayPathsInClip.Count} ボーン分）", GUILayout.Height(28)))
                        StripAndSave();
                }
            }

            if (!string.IsNullOrEmpty(lastReport))
            {
                scroll = EditorGUILayout.BeginScrollView(scroll);
                EditorGUILayout.TextArea(lastReport, GUILayout.ExpandHeight(true));
                EditorGUILayout.EndScrollView();
            }
        }

        /// <summary>揺れ物ボーンの相対パス集合を作る（GLBの extras.mmd 由来。定義は MmdEditorShared 参照）</summary>
        private Dictionary<string, string> CollectSwayBonePaths(out string error)
        {
            if (!MmdEditorShared.TryCollectSwayBones(modelRoot, out var bones, out error))
                return new Dictionary<string, string>();
            return MmdEditorShared.ToRelativePaths(bones, modelRoot.transform);
        }

        private static bool IsConstantCurve(AnimationCurve c)
        {
            if (c == null || c.keys.Length <= 1) return true;
            float v0 = c.keys[0].value;
            for (int i = 1; i < c.keys.Length; i++)
                if (Mathf.Abs(c.keys[i].value - v0) > 1e-5f) return false;
            return true;
        }

        private void Inspect()
        {
            var swayPaths = CollectSwayBonePaths(out string collectError);
            if (swayPaths.Count == 0)
            {
                // 揺れ物が1本も取れないまま「カーブ0本=無罪」と報告すると誤診になる。理由を出して止める。
                lastReport = "★検査できません: 揺れ物ボーンを特定できませんでした。\n  " + collectError;
                _swayPathsInClip.Clear();
                Debug.LogWarning("[クリップ診断] " + collectError);
                return;
            }
            var bindings = AnimationUtility.GetCurveBindings(clip);

            _swayPathsInClip.Clear();
            var byLabel = new Dictionary<string, int>();
            int swayCurveCount = 0, swayConstCount = 0;
            var samples = new List<string>();

            foreach (var b in bindings)
            {
                if (!swayPaths.TryGetValue(b.path, out string label)) continue;
                swayCurveCount++;
                _swayPathsInClip.Add(b.path);
                byLabel.TryGetValue(label, out int c); byLabel[label] = c + 1;
                var curve = AnimationUtility.GetEditorCurve(clip, b);
                bool isConst = IsConstantCurve(curve);
                if (isConst) swayConstCount++;
                if (samples.Count < 12)
                    samples.Add($"    {b.path} : {b.propertyName}（キー{curve.keys.Length}個, {(isConst ? "定数" : "動きあり")}）");
            }

            var sb = new StringBuilder();
            sb.AppendLine($"クリップ: {clip.name} ／ 総カーブ数: {bindings.Length}");
            sb.AppendLine($"揺れ物ボーン（extras.mmd の物理演算剛体 mode≠0 が紐づくボーン）: {swayPaths.Count} 本");
            sb.AppendLine();
            if (swayCurveCount == 0)
            {
                sb.AppendLine("★判定: 揺れ物ボーン向けのカーブは 0 本。クリップは無罪です。");
                sb.AppendLine("  → 上書きの犯人は別（Animator以外でTransformを書く何か）を探す必要があります。");
            }
            else
            {
                sb.AppendLine($"★判定: 揺れ物ボーン向けのカーブが {swayCurveCount} 本 見つかりました" +
                              $"（うち定数カーブ {swayConstCount} 本）。");
                sb.AppendLine("  内訳: " + string.Join(" / ", byLabel.Select(kv => $"{kv.Key}={kv.Value}本")));
                sb.AppendLine("  → 定数でもAnimatorは毎フレーム書き戻します。これがライブ物理を上書きし、");
                sb.AppendLine("     「数字は動くのに絵が鉄壁」の正体である可能性が濃厚です。");
                sb.AppendLine("     下のボタンで揺れ物カーブを除去した複製クリップを作り、Animatorに差し替えてください。");
                sb.AppendLine();
                sb.AppendLine("  サンプル（最大12件）:");
                foreach (var s in samples) sb.AppendLine(s);
            }
            lastReport = sb.ToString();
            Debug.Log("[クリップ診断]\n" + lastReport);
        }

        private void StripAndSave()
        {
            string srcPath = AssetDatabase.GetAssetPath(clip);
            var copy = Instantiate(clip);
            copy.name = clip.name + "_stripped";

            int removed = 0;
            foreach (var b in AnimationUtility.GetCurveBindings(copy).ToArray())
            {
                if (!_swayPathsInClip.Contains(b.path)) continue;
                AnimationUtility.SetEditorCurve(copy, b, null); // カーブ削除
                removed++;
            }

            string dir = string.IsNullOrEmpty(srcPath) ? "Assets" : System.IO.Path.GetDirectoryName(srcPath);
            string savePath = AssetDatabase.GenerateUniqueAssetPath($"{dir}/{copy.name}.anim");
            AssetDatabase.CreateAsset(copy, savePath);
            AssetDatabase.SaveAssets();

            lastReport += $"\n★削除完了: {removed} 本のカーブを除去し {savePath} に保存しました。\n" +
                          "  AnimatorのクリップをこのStripped版に差し替えて再生してください。\n" +
                          "  （体のカーブはそのまま＝ダンスは変わらず、揺れ物だけライブ物理に委ねられます）";
            Debug.Log($"[クリップ診断] {removed} 本のカーブを除去 → {savePath}");
        }
    }
}
