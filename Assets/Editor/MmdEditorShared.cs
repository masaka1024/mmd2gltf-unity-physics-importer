// ===========================================================================
// インポーター系エディタツールの共有ヘルパー。
//
// 「揺れ物ボーンとは何か」の定義を一箇所に集める。
//   旧: モデル配下の「非Kinematic な Rigidbody が乗っている Transform」
//        → PhysX 経路を撤去した(2026-08-10)ため常に 0 件になり、
//          クリップ診断もスキン結線検査も黙って「揺れ物なし」と答えるようになっていた。
//   新: GLB の extras.mmd にある PMX 剛体のうち physicsMode != 0 (=物理演算) が
//        紐づくボーン。これは PMX 作者が「揺れ物」として定義したものそのもので、
//        自作Bulletエンジンが実際に駆動する集合とも一致する。
//        Unity 側に何が付いているか(Rigidbody の有無)に依存しないのが利点。
// ===========================================================================
using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using BulletPhysics.Pmx;

namespace Mmd2GltfImporter
{
    public static class MmdEditorShared
    {
        /// <summary>Scene 上のモデルから、元になった .glb アセットのパス(Assets 相対)を解決する。
        /// GLB 由来でない・解決できない場合は null。</summary>
        public static string ResolveGlbAssetPath(GameObject target)
        {
            if (target == null) return null;

            string assetPath = "";
            GameObject prefabRoot = PrefabUtility.GetOutermostPrefabInstanceRoot(target);
            if (prefabRoot != null)
            {
                UnityEngine.Object sourceAsset = PrefabUtility.GetCorrespondingObjectFromSource(prefabRoot);
                if (sourceAsset != null) assetPath = AssetDatabase.GetAssetPath(sourceAsset);
            }
            if (string.IsNullOrEmpty(assetPath))
            {
                var originalSource = PrefabUtility.GetCorrespondingObjectFromSource(target);
                if (originalSource != null) assetPath = AssetDatabase.GetAssetPath(originalSource);
                else assetPath = AssetDatabase.GetAssetPath(target);
            }

            if (string.IsNullOrEmpty(assetPath) ||
                !assetPath.EndsWith(".glb", StringComparison.OrdinalIgnoreCase)) return null;
            return assetPath;
        }

        /// <summary>剛体名から部位ラベルを付ける（レポートの内訳表示用）。</summary>
        public static string ClassifyPart(string name)
        {
            if (string.IsNullOrEmpty(name)) return "その他揺れ物";
            if (name.Contains("スカート") || name.Contains("Skirt")) return "スカート";
            if (name.Contains("髪") || name.Contains("もみあげ") || name.Contains("モミアゲ") ||
                name.Contains("Hair") || name.Contains("TwinTails") || name.Contains("Ribbon")) return "髪";
            return "その他揺れ物";
        }

        /// <summary>
        /// モデル配下の「揺れ物ボーン」を GLB の extras.mmd から集める。
        /// 戻り値のキーはボーン Transform、値は部位ラベル。
        /// </summary>
        /// <param name="modelRoot">Scene 上のモデルルート</param>
        /// <param name="result">揺れ物ボーン → 部位ラベル</param>
        /// <param name="error">失敗理由（成功時 null）。UI にそのまま出せる文面にしてある。</param>
        public static bool TryCollectSwayBones(GameObject modelRoot,
                                               out Dictionary<Transform, string> result,
                                               out string error)
        {
            result = new Dictionary<Transform, string>();
            error = null;

            if (modelRoot == null) { error = "モデルが指定されていません。"; return false; }

            string glb = ResolveGlbAssetPath(modelRoot);
            if (string.IsNullOrEmpty(glb))
            {
                error = "元の .glb アセットが見つかりません。対象は GLB から取り込んだモデルにしてください。";
                return false;
            }

            PmxPhysicsModel model;
            try
            {
                model = GlbPhysicsReader.LoadFile(System.IO.Path.GetFullPath(glb), out _, out _);
            }
            catch (Exception e)
            {
                error = $"GLB の物理データを読めませんでした: {e.Message}";
                return false;
            }

            // ボーン名 → Transform（エンジンと同じく名前で解決する）
            var byName = new Dictionary<string, Transform>();
            foreach (var t in modelRoot.GetComponentsInChildren<Transform>(true))
                if (!byName.ContainsKey(t.name)) byName[t.name] = t;

            int unresolved = 0;
            foreach (var rb in model.RigidBodies)
            {
                if (rb.PhysicsMode == 0) continue;                 // 0 = ボーン追従(体パーツ) は揺れ物ではない
                if (rb.BoneIndex < 0 || rb.BoneIndex >= model.BoneNames.Count) continue; // 錘などボーン無し剛体
                string boneName = model.BoneNames[rb.BoneIndex];
                if (byName.TryGetValue(boneName, out var tr) && tr != null)
                    result[tr] = ClassifyPart(rb.Name);
                else
                    unresolved++;
            }

            if (result.Count == 0)
            {
                error = unresolved > 0
                    ? $"揺れ物剛体は見つかりましたが、対応するボーンが {unresolved} 件ともモデル配下にありません。" +
                      "対象のモデルルートが正しいか確認してください。"
                    : "この GLB の extras.mmd には物理演算(mode≠0)の剛体がありません。揺れ物が定義されていないモデルです。";
                return false;
            }
            if (unresolved > 0)
                Debug.LogWarning($"[MMD] 揺れ物剛体 {unresolved} 件はボーンを解決できず除外しました。");
            return true;
        }

        /// <summary>Transform → modelRoot からの相対パス（AnimationClip のバインディングと同じ形式）。</summary>
        public static Dictionary<string, string> ToRelativePaths(Dictionary<Transform, string> bones, Transform root)
        {
            var paths = new Dictionary<string, string>();
            foreach (var kv in bones)
            {
                string p = AnimationUtility.CalculateTransformPath(kv.Key, root);
                paths[p] = kv.Value;
            }
            return paths;
        }
    }
}
