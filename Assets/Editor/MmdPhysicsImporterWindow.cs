using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using BulletPhysics.Unity; // 自作Bulletエンジンの Unity ブリッジ（MmdPhysicsBehaviour / MmdPhysicsBackendSwitch）

// MMD glTF 統合インポーター。GLB(extras.mmd)を読み、Unity上のモデルに対して
//   【1】物理を配線する（自作Bulletエンジン MmdPhysicsBehaviour）
//   【2】マテリアルをlilToonへ変換する（トゥーン/スフィア/輪郭線の復元）
// の2つを行う。加えて、GLBの生JSONを覗くデバッグダンプを持つ。
//
// ★2026-08-10：PhysX経路を撤去した。
//   かつてこのウィンドウの主機能は「Unity Rigidbody + ConfigurableJoint をボーンに直付けして
//   PhysXで揺らす」ことだった(mmd-for-unity 方式)。自作BulletエンジンがMMD / PmxEditorの
//   挙動に到達したため主経路を入れ替え、さらに二重メンテを避けるため PhysX 側を削除した。
//   これに伴い以下を除去:
//     - 剛体生成 / ジョイント結合 / コライダー配置 / レイヤー管理 (約1,900行)
//     - 調整パネルの tune_* 49個 (ばね・減衰・ソフトリミット・部位別ダイヤル等。すべてPhysX専用だった)
//     - PhysX前提の検証・診断 (ValidatePhysicsSetup / ValidateRestOverlap / DiagnosePhysics 等)
//     - 補助ランタイム (MmdGravity / MmdPhysicsWarmup / MmdCollisionMask / MmdJointProbe 等)
//   物理側の調整は MmdPhysicsBehaviour の Inspector に一本化されている。
//   撤去前の全文は git 履歴、または当時のバックアップを参照すること。
namespace Mmd2GltfImporter
{
    public class MmdPhysicsImporterWindow : EditorWindow
    {
        // メニューは "MMD Physics/" に統一（従来は本体だけ "MMD/" 配下で分裂していた）。
        // priority で 本体 → 区切り線 → 診断ツール の順に並べる。
        [MenuItem("MMD Physics/インポーター", false, 0)]
        public static void ShowWindow()
        {
            GetWindow<MmdPhysicsImporterWindow>("MMD Physics");
        }

        [SerializeField] private GameObject targetPrefab;
        [SerializeField] private bool useEnglish = false; // UIラベルの言語切り替え（Consoleログは対象外）

        // ★GLBバイナリから画像を直接抽出するための作業キャッシュ（マテリアル変換時のみ使用）
        private byte[] glbBytesCache;
        private List<GltfBufferView> bufferViewsCache;
        private List<GltfImageData> imagesCache;
        private List<GltfTextureData2> gltfTexturesCache;
        private int binChunkStart = -1;
        private Dictionary<int, Texture2D> extractedTextureCache = new Dictionary<int, Texture2D>();
        private Dictionary<int, Texture2D> sharedToonCache = new Dictionary<int, Texture2D>();

        // ★ウィンドウ状態と、残った数少ない調整値。
        //   PhysX経路の撤去(2026-08-10)に伴い、旧「調整パネル」の tune_* 49個は
        //   すべて削除した。物理側の調整は MmdPhysicsBehaviour の Inspector にある。
        private Vector2 scrollPos;                                        // ウィンドウ全体のスクロール位置
        [SerializeField] private bool showSecMaterial = false;            // マテリアル調整の小見出し
        [SerializeField] private bool showDebugPanel = false;

        [SerializeField] private float tune_outlineWidthFactor = 0.08f;   // edgeSize→lilToon _OutlineWidthへの換算係数

        // ★ゲーム内の当たり判定（【3】）。物理ではなく判定専用のコライダーを作る設定。
        [SerializeField] private MmdHitboxBuilder.Scope hit_scope = MmdHitboxBuilder.Scope.BodyOnly;
        [SerializeField] private string hit_layerName = "MMDHitbox";
        [SerializeField] private bool   hit_asTrigger = true;

        private void OnGUI()
        {
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label(L("MMD glTF 統合インポーター (物理=自作Bulletエンジン)", "MMD glTF Unity Importer (Physics = Custom Bullet Engine)"), EditorStyles.boldLabel);
            GUILayout.FlexibleSpace();
            useEnglish = GUILayout.Toggle(useEnglish, useEnglish ? "EN" : "日本語", "Button", GUILayout.Width(60));
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.Space();

            // ★ウィンドウが縦に伸びても操作できるよう全体をスクロールさせる
            scrollPos = EditorGUILayout.BeginScrollView(scrollPos);

            targetPrefab = (GameObject)EditorGUILayout.ObjectField(
                L("対象モデル (Scene上)", "Target Model (in Scene)"), targetPrefab, typeof(GameObject), true);

            EditorGUILayout.Space();

            // ═══════════════════════════════════════════
            //  【1】自作Bullet物理（唯一の物理エンジン）
            //  GLBの extras.mmd から剛体・ジョイント・ボーンを直接構築する。このボタンだけで物理は動く。
            // ═══════════════════════════════════════════
            GUILayout.Label(L("【1】物理（自作Bulletエンジン）", "[1] Physics (Custom Bullet Engine)"), EditorStyles.miniBoldLabel);
            if (GUILayout.Button(L("1. 物理を配線 / 再配線 (MmdPhysicsBehaviour + BackendSwitch)", "1. Wire / Re-wire Physics (MmdPhysicsBehaviour + BackendSwitch)"))
                && targetPrefab != null)
            {
                AttachCustomEngine();
            }
            EditorGUILayout.HelpBox(
                L("GLBの extras.mmd から剛体・ジョイント・ボーンを直接構築します。" +
                  "刻み・貫入対策などの細かい設定は、配線後に付く MmdPhysicsBehaviour の Inspector で調整してください。",
                  "Builds bodies/joints/bones straight from the GLB's extras.mmd. " +
                  "Fine-tuning (timestep, penetration fixes) lives on the MmdPhysicsBehaviour component added by this button."),
                MessageType.Info);

            EditorGUILayout.Space();

            // ═══════════════════════════════════════════
            //  【2】描画・見た目
            // ═══════════════════════════════════════════
            GUILayout.Label(L("【2】描画・見た目の修正", "[2] Rendering & Appearance"), EditorStyles.miniBoldLabel);
            if (GUILayout.Button(L("2. マテリアルをlilToonへ変換（トゥーン/スフィア復元）", "2. Convert Materials to lilToon (restore Toon/Sphere maps)")) && targetPrefab != null)
            {
                ConvertMaterialsToLilToon();
            }
            showSecMaterial = EditorGUILayout.Foldout(showSecMaterial, L("マテリアルの調整", "Material Tuning"), true);
            if (showSecMaterial)
            {
                EditorGUI.indentLevel++;
                tune_outlineWidthFactor = EditorGUILayout.Slider(L("輪郭線の太さ換算係数", "Outline Width Factor"), tune_outlineWidthFactor, 0.01f, 0.3f);
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.Space();

            // ═══════════════════════════════════════════
            //  【3】ゲーム内の当たり判定（ヒットボックス）
            //  PMXの剛体定義からコライダーだけを作る。物理は増えない（Rigidbodyなし）。
            // ═══════════════════════════════════════════
            GUILayout.Label(L("【3】ゲーム内の当たり判定（ヒットボックス）", "[3] In-Game Hitboxes"), EditorStyles.miniBoldLabel);
            hit_scope = (MmdHitboxBuilder.Scope)EditorGUILayout.Popup(L("生成する範囲", "Scope"), (int)hit_scope,
                useEnglish ? MmdHitboxBuilder.ScopeLabelsEn : MmdHitboxBuilder.ScopeLabelsJa);
            hit_layerName = EditorGUILayout.TextField(L("レイヤー名", "Layer Name"), hit_layerName);
            hit_asTrigger = EditorGUILayout.Toggle(L("トリガーにする（推奨）", "Make Triggers (recommended)"), hit_asTrigger);

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button(L("3. 当たり判定を生成 / 作り直し", "3. Build / Rebuild Hitboxes")) && targetPrefab != null)
            {
                string glb = GetGlbAssetPath();
                if (string.IsNullOrEmpty(glb))
                    Debug.LogWarning("[MMD Hitbox] 元の .glb アセットが見つかりません（対象はGLB由来のモデルにしてください）。");
                else
                    MmdHitboxBuilder.Build(targetPrefab, System.IO.Path.GetFullPath(glb), hit_scope, hit_layerName, hit_asTrigger);
            }
            using (new EditorGUI.DisabledScope(targetPrefab == null || MmdHitboxBuilder.Count(targetPrefab) == 0))
                if (GUILayout.Button(L("除去", "Remove"), GUILayout.Width(80)) && targetPrefab != null)
                    MmdHitboxBuilder.Remove(targetPrefab);
            EditorGUILayout.EndHorizontal();

            if (targetPrefab != null)
            {
                int n = MmdHitboxBuilder.Count(targetPrefab);
                EditorGUILayout.HelpBox(
                    (n > 0
                        ? L($"現在 {n} 個のヒットボックスが付いています。", $"{n} hitboxes currently attached.")
                        : L("まだ付いていません。", "None attached yet.")) +
                    L("　コライダーをボーンの子に置くだけなので物理計算は増えず、アニメでも自作エンジンでもボーンに追従します。" +
                      "当たった部位は MmdHitbox.PartName（頭・上半身2 など）で判別できます。",
                      "  Colliders are parented to bones only — no added physics cost; they follow whether driven by the Animator or the custom engine. " +
                      "Identify the part hit via MmdHitbox.PartName."),
                    n > 0 ? MessageType.None : MessageType.Info);
            }

            EditorGUILayout.Space();

            showDebugPanel = EditorGUILayout.Foldout(showDebugPanel, L("【デバッグ】", "[Debug]"), true);
            if (showDebugPanel)
            {
                EditorGUI.indentLevel++;
                if (GUILayout.Button(L("ジョイントJSONを出力（キー名の確認用）", "Dump Joints JSON (check key names)")) && targetPrefab != null)
                {
                    DumpJointsJson();
                }
                if (GUILayout.Button(L("指定キーワードのボーン回転を一括出力（再生中/一時停止中も可）", "Dump Bone Rotations by Keyword (works during Play/Pause)")) && targetPrefab != null)
                {
                    DumpBoneRotationsByKeyword("スカート");
                }


                if (GUILayout.Button(L("マテリアル(extras.mmd.materials)の生JSONを出力", "Dump Raw Materials JSON (extras.mmd.materials)")) && targetPrefab != null)
                {
                    DumpMaterialsJson();
                }

                if (GUILayout.Button(L("テクスチャ実体とimages/texturesの対応を確認", "Check Texture Asset ↔ images/textures Mapping")) && targetPrefab != null)
                {
                    DumpTextureAssets();
                }
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.EndScrollView();
        }

        // ★UIラベル用の簡易ローカライズヘルパー。useEnglishがtrueなら英語、falseなら日本語を返す。
        //   Consoleログ(Debug.Log等)はこの対象外で、日本語のまま。
        private string L(string ja, string en) => useEnglish ? en : ja;

        // ═══════════════════════════════════════════
        //  JSON 読み込み系
        // ═══════════════════════════════════════════
        // targetPrefab の元となる .glb アセットのパス（Assets 相対）を解決する。
        // 実体は MmdEditorShared（クリップ診断・スキン結線検査と共有）。
        private string GetGlbAssetPath() => MmdEditorShared.ResolveGlbAssetPath(targetPrefab);

        private string GetRawJsonText()
        {
            string assetPath = GetGlbAssetPath();
            if (string.IsNullOrEmpty(assetPath)) return null;

            try
            {
                byte[] bytes = File.ReadAllBytes(assetPath);
                if (bytes.Length < 20) return null;
                int chunkLength = BitConverter.ToInt32(bytes, 12);
                return Encoding.UTF8.GetString(bytes, 20, chunkLength);
            }
            catch { return null; }
        }

        // ═══════════════════════════════════════════
        //  自作Bulletエンジンの配線 ＝【1】物理の全体
        //
        //  ・targetPrefab のルートに MmdPhysicsBehaviour（GLBの extras.mmd から
        //    剛体/Joint/ボーンを構築し、実行時にボーンを駆動）を付与・設定する。これだけで物理は成立する。
        //  ・MmdPhysicsBackendSwitch も併せて付ける。PhysX経路を撤去した今、切替先は無く
        //    Mode は常に Custom だが、外部から持ち込まれた Rigidbody が混ざった場合に
        //    ボーンの奪い合いを防ぐ保険として残してある（対象が無ければ空転する）。
        //  ・冪等: 既に付いていれば再設定する。
        // ═══════════════════════════════════════════
        private void AttachCustomEngine()
        {
            if (targetPrefab == null) return;

            string glbPath = GetGlbAssetPath();
            if (string.IsNullOrEmpty(glbPath))
            {
                Debug.LogWarning("[MMD Physics] 元の .glb アセットが見つからないため自作エンジンの配線をスキップしました（対象はGLB由来のモデルにしてください）。");
                return;
            }
            // GlbPhysicsReader は File.ReadAllBytes(path) で読むため、実行時の作業ディレクトリに
            // 依存しない絶対パスを渡す（Assets 相対でもエディタでは解決可能だが、より堅牢に）。
            string absPath = System.IO.Path.GetFullPath(glbPath);

            GameObject root = targetPrefab;

            // ── 1) 自作エンジン駆動コンポーネント ──
            var behaviour = root.GetComponent<MmdPhysicsBehaviour>();
            if (behaviour == null) behaviour = Undo.AddComponent<MmdPhysicsBehaviour>(root);
            Undo.RecordObject(behaviour, "Configure MmdPhysicsBehaviour");
            behaviour.Source = MmdPhysicsBehaviour.InputSource.Glb;
            behaviour.GlbPath = absPath;
            behaviour.ModelRoot = root.transform; // ボーンTransformは名前でこの配下から解決される

            // ── 2) 排他切替コンポーネント ──
            var sw = root.GetComponent<MmdPhysicsBackendSwitch>();
            if (sw == null) sw = Undo.AddComponent<MmdPhysicsBackendSwitch>(root);
            Undo.RecordObject(sw, "Configure MmdPhysicsBackendSwitch");
            sw.customEngine = behaviour;
            sw.Mode = MmdPhysicsBackendSwitch.Backend.Custom;

            EditorUtility.SetDirty(root);
            Debug.Log($"[MMD Physics] 自作Bulletエンジンを配線しました。GLB={glbPath}。再生すると extras.mmd から物理を構築してボーンを駆動します（細かい設定は MmdPhysicsBehaviour の Inspector）。");
        }

        // ★オブジェクト({)を含む配列だけを探す。整数だけの配列(skinのjoints)は飛ばす。
        private string FindJsonArrayObjects(string json, string key)
        {
            int searchFrom = 0;
            while (true)
            {
                int keyIndex = json.IndexOf("\"" + key + "\"", searchFrom);
                if (keyIndex == -1) return null;

                int startBracket = json.IndexOf("[", keyIndex);
                if (startBracket == -1) return null;

                int bracketCount = 1;
                int end = -1;
                for (int i = startBracket + 1; i < json.Length; i++)
                {
                    if (json[i] == '[') bracketCount++;
                    else if (json[i] == ']')
                    {
                        bracketCount--;
                        if (bracketCount == 0) { end = i; break; }
                    }
                }
                if (end == -1) return null;

                string arr = json.Substring(startBracket, end - startBracket + 1);
                if (arr.IndexOf('{') != -1) return arr; // オブジェクトを含む＝本命
                searchFrom = end + 1;                    // 整数だけ＝skin側なので次を探す
            }
        }

        // ★剛体名から部位カテゴリを判定する（ジョイント名(jd.name)は剛体名と一致しない
        //   ケースがあるため使わない。既存のスカート/髪判定(isSkirt/isHair)も剛体名ベース）。
        //   PMXのボーン／剛体命名慣習に合わせ、まず「前髪」「もみあげ／モミアゲ」を
        //   先にチェックしてから「スカート」を見る（"髪"を含む他の部位と混同しないため、
        //   ここでは部分一致のみを使い、"髪"全体は対象にしない＝後ろ髪等は既定の全身共通値のまま）。
        // ═══════════════════════════════════════════
        //  マテリアル修正（従来どおり）
        // ═══════════════════════════════════════════
        // ═══════════════════════════════════════════
        //  マテリアル変換（★lilToonへ変換・トゥーン/スフィア復元版）
        //
        //  glTFの標準"materials"配列(pbrMetallicRoughness/alphaMode等)と、
        //  extras.mmd（MMD固有のsphereMode/sphereTexture/toonTexture）を読み、
        //  対応するUnity上のMaterial/Texture2Dを「名前」で引き当てて
        //  lilToonのプロパティへ変換する。
        //
        //  ※sphereTexture/toonTextureはglTFの"textures"配列の番号だが、
        //    Unity側にサブアセットとしてインポートされたTexture2Dの並び順は
        //    glTFの配列順と一致しない（実物で確認済み）。名前は完全一致するため、
        //    必ず名前で照合する（ボーンの時と同じ教訓）。
        private void ConvertMaterialsToLilToon()
        {
            // ★lilToon未導入なら、無駄な処理をする前に最初に弾く。Consoleのエラーは
            //   見落とされやすいため、確実に気づけるようダイアログでも表示する。
            Shader lilShader = Shader.Find("lilToon");
            if (lilShader == null)
            {
                Debug.LogError("[MMD Material] lilToonシェーダーが見つかりません。lilToonをプロジェクトに導入してください。");
                EditorUtility.DisplayDialog(
                    "lilToonが見つかりません",
                    "マテリアル変換にはlilToonが必要です。\nプロジェクトにlilToonを導入してから、もう一度お試しください。",
                    "OK");
                return;
            }

            string json = GetRawJsonText();
            if (string.IsNullOrEmpty(json))
            {
                Debug.LogError("[MMD Material] JSONを取得できませんでした。対象がGLBか確認してください。");
                return;
            }

            // ① materials配列を解析
            string materialsArr = FindJsonArrayObjects(json, "materials");
            List<GltfMaterialData> materialList = new List<GltfMaterialData>();
            if (!string.IsNullOrEmpty(materialsArr))
            {
                var wrapper = JsonUtility.FromJson<GltfMaterialListWrapper>("{\"list\":" + materialsArr + "}");
                if (wrapper != null && wrapper.list != null) materialList = wrapper.list;
            }
            if (materialList.Count == 0)
            {
                Debug.LogWarning("[MMD Material] materials配列が読めませんでした。");
                return;
            }

            // ② textures配列から「番号→本当のテクスチャ名」を作る
            List<string> textureNames = ParseNamedArray(json, "textures");

            // ③ 同じ.glbアセットの中から、Material/Texture2Dのサブアセットを名前で引けるようにする
            string assetPath = GetSourceAssetPath();
            if (string.IsNullOrEmpty(assetPath))
            {
                Debug.LogError("[MMD Material] アセットパスが特定できませんでした。");
                return;
            }
            var allSubAssets = AssetDatabase.LoadAllAssetsAtPath(assetPath);
            var materialsByName = new Dictionary<string, Material>();
            var texturesByName = new Dictionary<string, Texture2D>();
            foreach (var obj in allSubAssets)
            {
                if (obj is Material m && !materialsByName.ContainsKey(m.name)) materialsByName[m.name] = m;
                if (obj is Texture2D t && !texturesByName.ContainsKey(t.name)) texturesByName[t.name] = t;
            }

            // ★UniGLTF の「Extract Materials And Textures ...」を実行すると、Material/Texture2D は
            //   .glb のサブアセットではなくなり、外部ファイルへ remap される (importer の externalObjects)。
            //   この状態では上の LoadAllAssetsAtPath が 0件を返し、**全マテリアルが
            //   「対応するUnity Materialが見つからず」でスキップされる**。
            //   (Unity 6 で非圧縮 ARGB32 のサブアセットを避けるために抽出が要るので、両立させる)
            //   キーには remap の SourceAssetIdentifier 側の名前を使う。抽出時にファイル名が
            //   衝突回避で改名される (例 "__UNIGLTF__DUPLICATED__") ことがあり、
            //   アセット名では glTF の materials/textures 配列と突き合わせられないため。
            //   ★抽出していない場合 externalObjects は空なので、従来と完全に同じ動作になる。
            var glbImporter = AssetImporter.GetAtPath(assetPath);
            // ★抽出(remap)が有効かどうか。有効なときは .glb が返すサブアセットのテクスチャを信用しない
            //   (下の FindOrExtractTextureByIndex を参照)。
            bool hasExtraction = glbImporter != null && glbImporter.GetExternalObjectMap().Count > 0;
            if (glbImporter != null)
            {
                foreach (var kv in glbImporter.GetExternalObjectMap())
                {
                    string key = kv.Key.name;
                    if (string.IsNullOrEmpty(key)) continue;
                    if (kv.Value is Material em && !materialsByName.ContainsKey(key)) materialsByName[key] = em;
                    if (kv.Value is Texture2D et && !texturesByName.ContainsKey(key)) texturesByName[key] = et;
                }
            }

            Texture2D FindTextureByIndex(int texIndex)
            {
                if (texIndex < 0 || texIndex >= textureNames.Count) return null;
                string texName = textureNames[texIndex];
                if (string.IsNullOrEmpty(texName)) return null;
                return texturesByName.TryGetValue(texName, out var tex) ? tex : null;
            }

            // ★トゥーン/スフィア用：まず既存のUnityサブアセットを名前で探し（標準スロット用途で
            //   たまたま既にインポートされているケースに備える）、無ければGLBバイナリから
            //   直接抽出する。UniGLTFは標準マテリアルから参照されない画像をインポートしないため。
            PrepareGlbBinaryAccess(json, assetPath);
            Texture2D FindOrExtractTextureByIndex(int texIndex)
            {
                var existing = FindTextureByIndex(texIndex);
                // ★UniGLTF の Extract を掛けると、remap されたテクスチャについても
                //   AssetDatabase.LoadAllAssetsAtPath(.glb) は Texture2D を返してくるが、
                //   **その実体は既に無い(ダングリング)**。代入するとマテリアルが真っ白になる。
                //   抽出が有効なときは .glb 所有のサブアセットを信用せず、GLBバイナリから取り直す。
                //   実測: 抽出済みの11枚だけが白くなり、未抽出のものは元からバイナリ経路に
                //   落ちていたため無事だった (2026-08-25 の診断ログで確定)。
                //   ★抽出していない場合 hasExtraction=false なので、従来と完全に同じ動作。
                if (existing != null && hasExtraction
                    && AssetDatabase.GetAssetPath(existing) == assetPath) existing = null;
                if (existing != null) return existing;
                return ExtractTextureFromGlb(texIndex, assetPath);
            }

            int converted = 0, skipped = 0, sphereApplied = 0, toonApplied = 0, outlineApplied = 0;

            for (int slotIdx = 0; slotIdx < materialList.Count; slotIdx++)
            {
                var md = materialList[slotIdx];
                if (string.IsNullOrEmpty(md.name) || !materialsByName.TryGetValue(md.name, out Material mat))
                {
                    Debug.LogWarning($"[MMD Material] マテリアル '{md.name}' に対応するUnity Materialが見つからずスキップしました。");
                    skipped++;
                    continue;
                }

                Undo.RecordObject(mat, "Convert to lilToon");

                // ── レンダリングモード判定（Opaque/Cutout/Transparent）──
                // 0=Opaque, 1=Cutout, 2=Transparent（lilToon.RenderingMode の並び順と一致）
                int modeInt = 0;
                if (md.alphaMode == "MASK") modeInt = 1;
                else if (md.alphaMode == "BLEND") modeInt = 2;

                // ── MMD固有データを先に取得（輪郭線フラグをレンダリングモード設定時に使うため）──
                var mmd = md.extras != null ? md.extras.mmd : null;

                // ★輪郭線フラグ：MMDのflagsビットマスクのbit4(値16)が「このマテリアルは
                //   輪郭線を描画する」を意味する。
                bool hasOutline = mmd != null && (mmd.flags & 16) != 0;

                // ★元テクスチャ温存(origTexture)対応：
                //   エクスポーターはビューア安全のためプリベイク版(αを0/255に平坦化した
                //   テクスチャ)を標準のbaseColorTextureに使う一方、プリベイク前の無加工版を
                //   origTextureとしてGLBに同梱している。無加工版がある場合はそれを
                //   実アルファブレンド(半透明+TwoPass)で使い、MMD本来の透け髪・柔らかい
                //   眉・目の縁取りを再現する。
                bool useOrigTexture = mmd != null && mmd.origTexture >= 0;
                if (useOrigTexture) modeInt = 2; // MASK→Transparentへ昇格
                bool isCutout = (modeInt == 1);

                // ★alphaClass(本来のα分類)："blend"=真の半透明(透け髪・髪影・チーク等)。
                //   無印/"mask"はカットアウト由来の昇格組(肌・服・メガネ等、見た目は
                //   ほぼ不透明)。旧GLBはalphaClass未記載(null)だが、その場合は
                //   mask扱いにしておけば従来より安全側になる。
                bool trueBlend = md.alphaMode == "BLEND"
                    || (mmd != null && mmd.alphaClass == "blend");

                // ★半透明(BLEND)かつテクスチャありはTwoPass(2)：Normal(0)は深度書き込みが
                //   無く、髪の房や顔パーツの重なりで描画順が狂いグレーの筋状アーティファクトが
                //   出るため(モデルAで実機確認済み)。テクスチャ無しの半透明(レンズ等の
                //   単色ガラス)は従来どおりNormalでよい。
                //   lilToon.TransparentMode: 0=Normal, 1=OnePass, 2=TwoPass
                int transparentMode = 0;
                bool hasBaseTex = md.pbrMetallicRoughness != null
                    && md.pbrMetallicRoughness.baseColorTexture != null;
                if (modeInt == 2 && (hasBaseTex || useOrigTexture)) transparentMode = 2;

                // シェーダー未設定(初回)ならまずlilToonを割り当ててから正規セットアップを呼ぶ
                if (mat.shader != lilShader) mat.shader = lilShader;
                SetupLilToonRenderingMode(mat, modeInt, transparentMode, hasOutline);

                // ★レンダーキュー制御(半透明越しの消失バグ対策)：
                //   共有テクスチャのモデル(モデルB等)では全マテリアルにorigTextureが
                //   付き、全員がTransparent(q3000)へ昇格してしまう。透明キュー内は
                //   サブメッシュ番号順+深度書き込み(TwoPass)で描画されるため、
                //   「前髪(若い番号)の向こう側にあるメガネ(大きい番号)が深度テストに
                //   落ちて消える」症状が起きる(モデルBで実測確認)。
                //   対策：
                //   ・mask由来の昇格組(見た目ほぼ不透明) → AlphaTest帯(2452+slot)。
                //     深度を書きつつ真の半透明より先に描画されるので、透け髪や
                //     レンズ越しに正しく見える。ブレンド自体はキューに依らず有効。
                //   ・真の半透明(alphaClass=blend) → 3000+slot。MMDは材質順で
                //     描画するため、スロット順を維持してMMDの重なり順を再現する。
                if (modeInt == 2)
                {
                    mat.renderQueue = trueBlend
                        ? 3000 + slotIdx
                        : Mathf.Min(2452 + slotIdx, 2499);
                }

                // ── ベースカラー ──
                if (md.pbrMetallicRoughness != null)
                {
                    var c = md.pbrMetallicRoughness.baseColorFactor;
                    if (c != null && c.Length >= 4) mat.SetColor("_Color", new Color(c[0], c[1], c[2], c[3]));

                    if (md.pbrMetallicRoughness.baseColorTexture != null)
                    {
                        // ★FindOrExtract を使う (素の FindTextureByIndex ではない)。
                        //   ベースカラーは「標準マテリアルから参照されるので必ずサブアセットにある」
                        //   という前提で素の方を使っていたが、UniGLTF の Extract を掛けると
                        //   サブアセットが無くなりこの前提が崩れる。トゥーン/スフィア/orig と
                        //   同じくGLBバイナリ直読みのフォールバックに乗せる。
                        var tex = FindOrExtractTextureByIndex(md.pbrMetallicRoughness.baseColorTexture.index);
                        if (tex != null) mat.SetTexture("_MainTex", tex);
                    }
                }

                // ★無加工の元テクスチャがあれば_MainTexを差し替える
                //   (プリベイク版は使わず、実アルファを持つ元画像でブレンドする)。
                //   origテクスチャは標準マテリアルから参照されないためUnityの
                //   glTFインポートには含まれず、GLBバイナリから直接抽出する経路
                //   (FindOrExtractTextureByIndex)に自然に乗る。
                if (useOrigTexture)
                {
                    var origTex = FindOrExtractTextureByIndex(mmd.origTexture);
                    if (origTex != null)
                    {
                        mat.SetTexture("_MainTex", origTex);
                    }
                    else
                    {
                        Debug.LogWarning($"[MMD Material] '{md.name}': origTexture={mmd.origTexture} の抽出に失敗しました。プリベイク版を使用します。");
                    }
                }

                // ── アルファ閾値（MASK時）──
                if (isCutout) mat.SetFloat("_Cutoff", md.alphaCutoff > 0 ? md.alphaCutoff : 0.5f);

                // ── 両面描画 ──
                mat.SetFloat("_Cull", md.doubleSided ? 0f : 2f); // 0=Off(両面), 2=Back(片面)

                // ── 輪郭線（アウトライン）──
                if (hasOutline && mmd != null)
                {
                    mat.SetFloat("_UseOutline", 1f);
                    if (mmd.edgeColor != null && mmd.edgeColor.Length >= 4)
                        mat.SetColor("_OutlineColor", new Color(mmd.edgeColor[0], mmd.edgeColor[1], mmd.edgeColor[2], mmd.edgeColor[3]));
                    // ★MMDのedgeSize(標準の太さ=1.0前後)を、lilToonの_OutlineWidth(Range 0〜1,
                    //   既定値0.08)に換算する。MMD標準の太さ(1.0)がlilToonの既定太さ(0.08)に
                    //   相当するとみなした簡易な比例換算。太すぎ/細すぎる場合は係数0.08を調整。
                    float outlineWidth = mmd.edgeSize > 0 ? mmd.edgeSize * tune_outlineWidthFactor : tune_outlineWidthFactor;
                    mat.SetFloat("_OutlineWidth", Mathf.Clamp01(outlineWidth));
                    outlineApplied++;
                }

                // ── MMD固有：トゥーンテクスチャ・スフィアマップ ──
                // ★診断ログ：mmdデータそのものがnullなのか、値は読めているのに
                //   テクスチャ照合で失敗しているのかを切り分けるため。
                if (mmd == null)
                {
                    Debug.LogWarning($"[MMD Material][診断] '{md.name}': extras.mmd の解析結果がnullです。");
                }
                else
                {
                    Debug.Log($"[MMD Material][診断] '{md.name}': sphereMode={mmd.sphereMode}, sphereTexture={mmd.sphereTexture}, toonTexture={mmd.toonTexture}, toonShared={mmd.toonShared}, hasOutline={hasOutline}");
                }

                if (mmd != null)
                {
                    if (mmd.toonTexture >= 0)
                    {
                        var toonTex = FindOrExtractTextureByIndex(mmd.toonTexture);
                        if (toonTex == null)
                            Debug.LogWarning($"[MMD Material][診断] '{md.name}': toonTexture={mmd.toonTexture} に対応するテクスチャ名が見つかりませんでした（textureNames.Count={textureNames.Count}）。");
                        if (toonTex != null)
                        {
                            mat.SetTexture("_ShadowColorTex", toonTex);
                            toonApplied++;
                        }
                    }
                    else if (mmd.toonShared >= 0)
                    {
                        // MMD共有トゥーン(toon01〜10.bmp)。PMX仕様ではtoonShared=0が
                        // toon01.bmp、9がtoon10.bmpに対応する(0始まり→1始まりのズレに注意)。
                        var sharedTex = FindSharedToonTexture(mmd.toonShared);
                        if (sharedTex != null)
                        {
                            mat.SetTexture("_ShadowColorTex", sharedTex);
                            toonApplied++;
                        }
                        else
                        {
                            Debug.LogWarning($"[MMD Material] '{md.name}' は共有トゥーン(toonShared={mmd.toonShared})を使用していますが、" +
                                              $"対応するtoon{(mmd.toonShared + 1):00}が見つからず復元できませんでした。");
                        }
                    }

                    // ★sphereMode=3(サブテクスチャ)はUV1で貼る追加テクスチャであり、
                    //   視線方向でサンプリングするMatCapとは仕組みが異なる。誤って
                    //   MatCap乗算にすると見た目が大きく崩れるため適用せずスキップする。
                    if (mmd.sphereTexture >= 0 && mmd.sphereMode == 3)
                    {
                        Debug.LogWarning($"[MMD Material] '{md.name}': sphereMode=3(サブテクスチャ)は未対応のためスフィアを適用しませんでした。");
                    }
                    else if (mmd.sphereTexture >= 0 && mmd.sphereMode > 0)
                    {
                        var sphereTex = FindOrExtractTextureByIndex(mmd.sphereTexture);
                        if (sphereTex == null)
                            Debug.LogWarning($"[MMD Material][診断] '{md.name}': sphereTexture={mmd.sphereTexture} に対応するテクスチャ名が見つかりませんでした（textureNames.Count={textureNames.Count}）。");
                        if (sphereTex != null)
                        {
                            mat.SetTexture("_MatCapTex", sphereTex);
                            mat.SetFloat("_UseMatCap", 1f);
                            // ★sphereMode: MMD標準のPMX仕様に合わせて 1=乗算(Mul), 2=加算(Add) に修正。
                            //   （以前は仕様書記載どおり逆に割り当てていたが、実機テストで
                            //   sphereMode=2の乗算合成によりハイライト用スフィアテクスチャが
                            //   ベースカラーを黒く潰す症状が確認されたため、標準PMX仕様の
                            //   並びに戻した）
                            //   lilToonの_MatCapBlendMode: 0=Normal,1=Add,2=Screen,3=Mul。
                            mat.SetFloat("_MatCapBlendMode", mmd.sphereMode == 2 ? 1f : 3f);
                            sphereApplied++;
                        }
                    }
                }

                EditorUtility.SetDirty(mat);

                // ★ここまでの変更は、まだ.glbの「インポーターが生成したサブアセット」に
                //   対して行っている。このままだと、次回の再インポート時にUniGLTFが
                //   マテリアルを作り直して変更が消えてしまう。独立した.matファイルとして
                //   保存し直し、メッシュの参照先をそちらへ差し替えることで永続化する。
                Material persistentMat = SaveAsStandaloneMaterial(mat, md.name, assetPath);
                ReassignRendererMaterial(mat, persistentMat);

                converted++;
            }

            AssetDatabase.SaveAssets();
            Debug.Log($"[MMD Material] {converted}件をlilToonへ変換しました（トゥーン:{toonApplied}件, スフィア:{sphereApplied}件, 輪郭線:{outlineApplied}件, スキップ:{skipped}件）。");
        }

        // モデルの元アセットパス(.glb)を取得する共通処理
        // ★リフレクションで lilToon.lilToonInspector.SetupMaterialWithRenderingMode を呼ぶ。
        //   lilToon.Editor アセンブリは "autoReferenced": false のため、通常の
        //   using lilToon; では型が見えない（アセンブリ定義ファイルの追加が必要になる）。
        //   それを避けるため、型名を文字列で指定するリフレクションで直接呼び出す。
        //   renderingMode: 0=Opaque, 1=Cutout, 2=Transparent
        //   transparentMode: 0=Normal
        //   hasOutline: trueならアウトライン込みの実体シェーダー(例: Hidden/lilToonOutline)へ切り替える。
        //   ★3引数版の簡易オーバーロードは isoutl 等を「共有の静的フィールド(直前にInspectorで
        //   触った状態等)」から補うため挙動が不安定。ここでは全引数を明示するオーバーロードを
        //   使い、ambient staticに依存しないようにする。
        private void SetupLilToonRenderingMode(Material mat, int renderingMode, int transparentMode, bool hasOutline)
        {
            var inspectorType = System.Type.GetType("lilToon.lilToonInspector, lilToon.Editor");
            var renderingModeType = System.Type.GetType("lilToon.RenderingMode, lilToon.Editor");
            var transparentModeType = System.Type.GetType("lilToon.TransparentMode, lilToon.Editor");

            if (inspectorType == null || renderingModeType == null || transparentModeType == null)
            {
                Debug.LogError("[MMD Material] lilToonの内部型が見つかりませんでした。lilToonが正しく導入されているか確認してください。");
                return;
            }

            object renderingModeVal = System.Enum.ToObject(renderingModeType, renderingMode);
            object transparentModeVal = System.Enum.ToObject(transparentModeType, transparentMode);

            var method = inspectorType.GetMethod(
                "SetupMaterialWithRenderingMode",
                System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static,
                null,
                new System.Type[] { typeof(Material), renderingModeType, transparentModeType, typeof(bool), typeof(bool), typeof(bool), typeof(bool) },
                null);

            if (method == null)
            {
                Debug.LogError("[MMD Material] SetupMaterialWithRenderingMode(7引数) メソッドが見つかりませんでした（lilToonのバージョン差異の可能性）。");
                return;
            }

            // isoutl=hasOutline, islite=false, istess=false, ismulti=false
            method.Invoke(null, new object[] { mat, renderingModeVal, transparentModeVal, hasOutline, false, false, false });
        }

        // ★GLBの"bufferViews"/"images"/"textures"配列を読み込み、生バイナリ抽出の準備をする。
        //   ConvertMaterialsToLilToon実行の最初に一度だけ呼ぶ。
        private void PrepareGlbBinaryAccess(string json, string assetPath)
        {
            glbBytesCache = File.ReadAllBytes(assetPath);
            binChunkStart = -1;

            // chunk0(JSON)の長さから、chunk1(BIN)のヘッダー位置を計算する
            int chunk0Length = BitConverter.ToInt32(glbBytesCache, 12);
            int chunk1HeaderOffset = 20 + chunk0Length;
            if (chunk1HeaderOffset + 8 <= glbBytesCache.Length)
            {
                // chunk1Length = BitConverter.ToInt32(glbBytesCache, chunk1HeaderOffset); // 境界チェック用途のみ
                binChunkStart = chunk1HeaderOffset + 8;
            }

            string bvArr = FindJsonArrayObjects(json, "bufferViews");
            bufferViewsCache = new List<GltfBufferView>();
            if (!string.IsNullOrEmpty(bvArr))
            {
                var w = JsonUtility.FromJson<GltfBufferViewListWrapper>("{\"list\":" + bvArr + "}");
                if (w != null && w.list != null) bufferViewsCache = w.list;
            }

            string imgArr = FindJsonArrayObjects(json, "images");
            imagesCache = new List<GltfImageData>();
            if (!string.IsNullOrEmpty(imgArr))
            {
                var w = JsonUtility.FromJson<GltfImageListWrapper>("{\"list\":" + imgArr + "}");
                if (w != null && w.list != null) imagesCache = w.list;
            }

            string texArr = FindJsonArrayObjects(json, "textures");
            gltfTexturesCache = new List<GltfTextureData2>();
            if (!string.IsNullOrEmpty(texArr))
            {
                var w = JsonUtility.FromJson<GltfTextureListWrapper2>("{\"list\":" + texArr + "}");
                if (w != null && w.list != null) gltfTexturesCache = w.list;
            }

            extractedTextureCache = new Dictionary<int, Texture2D>();
        }

        // ★textures配列の番号(textureIndex)から、glTFのimages→bufferViewを辿って
        //   GLBバイナリ内の生画像バイト列を切り出し、Assetsフォルダ内へ新規テクスチャとして
        //   保存する。UniGLTFが標準スロット以外で使われる画像をインポートしないため、
        //   トゥーン/スフィア用テクスチャはこの方法で自前で取り出す必要がある。
        private Texture2D ExtractTextureFromGlb(int textureIndex, string assetPath)
        {
            if (extractedTextureCache.TryGetValue(textureIndex, out var cached)) return cached;

            if (gltfTexturesCache == null || textureIndex < 0 || textureIndex >= gltfTexturesCache.Count) return null;
            int imageIndex = gltfTexturesCache[textureIndex].source;
            if (imagesCache == null || imageIndex < 0 || imageIndex >= imagesCache.Count) return null;

            var img = imagesCache[imageIndex];
            if (img.bufferView < 0 || bufferViewsCache == null || img.bufferView >= bufferViewsCache.Count) return null;
            if (binChunkStart < 0) return null;

            var bv = bufferViewsCache[img.bufferView];
            int start = binChunkStart + bv.byteOffset;
            int length = bv.byteLength;
            if (start < 0 || length <= 0 || start + length > glbBytesCache.Length)
            {
                Debug.LogWarning($"[MMD Material] テクスチャ番号{textureIndex}のバイナリ範囲が不正です。");
                return null;
            }

            byte[] imgBytes = new byte[length];
            Array.Copy(glbBytesCache, start, imgBytes, 0, length);

            string ext = ".png";
            if (!string.IsNullOrEmpty(img.mimeType) && img.mimeType.Contains("jpeg")) ext = ".jpg";

            string parentFolder = Path.GetDirectoryName(assetPath).Replace("\\", "/");
            string folder = parentFolder + "/MMD_ExtractedTextures";
            if (!AssetDatabase.IsValidFolder(folder))
            {
                AssetDatabase.CreateFolder(parentFolder, "MMD_ExtractedTextures");
            }

            string safeName = string.IsNullOrEmpty(img.name) ? $"tex_{textureIndex}" : img.name;
            foreach (char c in Path.GetInvalidFileNameChars()) safeName = safeName.Replace(c, '_');
            string filePath = $"{folder}/{safeName}{ext}";

            File.WriteAllBytes(filePath, imgBytes);
            AssetDatabase.ImportAsset(filePath);
            var tex = AssetDatabase.LoadAssetAtPath<Texture2D>(filePath);

            if (tex == null)
                Debug.LogWarning($"[MMD Material] '{filePath}' のテクスチャインポートに失敗しました。");

            extractedTextureCache[textureIndex] = tex;
            return tex;
        }

        // ★MMD標準の共有トゥーン(toon01.bmp〜toon10.bmp)を、プロジェクト内から名前で探す。
        //   toonSharedIndexは0始まり(0=toon01, 9=toon10)。ユーザーがAssets内のどこかに
        //   toon01.bmp等を配置している前提（配置場所は問わず、ファイル名で検索する）。
        private Texture2D FindSharedToonTexture(int toonSharedIndex)
        {
            if (sharedToonCache.TryGetValue(toonSharedIndex, out var cached)) return cached;

            string name = $"toon{(toonSharedIndex + 1):00}"; // 0→toon01, 9→toon10
            Texture2D found = null;

            string[] guids = AssetDatabase.FindAssets($"{name} t:Texture2D");
            foreach (var guid in guids)
            {
                string path = AssetDatabase.GUIDToAssetPath(guid);
                string fileName = Path.GetFileNameWithoutExtension(path);
                if (string.Equals(fileName, name, System.StringComparison.OrdinalIgnoreCase))
                {
                    found = AssetDatabase.LoadAssetAtPath<Texture2D>(path);
                    break;
                }
            }

            if (found == null)
                Debug.LogWarning($"[MMD Material] 共有トゥーン '{name}' がプロジェクト内に見つかりません。" +
                                  $"Assets内のどこかに {name}.bmp（MMD標準の共有トゥーン画像）を配置してください。");

            sharedToonCache[toonSharedIndex] = found;
            return found;
        }

        // ★変換済みマテリアルを、独立した.matファイルとして保存する（無ければ新規作成、
        //   既にあれば中身だけ更新して使い回す＝再実行してもファイルが増殖しない）。
        private Material SaveAsStandaloneMaterial(Material sourceMat, string materialName, string glbAssetPath)
        {
            string parentFolder = Path.GetDirectoryName(glbAssetPath).Replace("\\", "/");
            string folder = parentFolder + "/MMD_Materials";
            if (!AssetDatabase.IsValidFolder(folder))
            {
                AssetDatabase.CreateFolder(parentFolder, "MMD_Materials");
            }

            string safeName = string.IsNullOrEmpty(materialName) ? sourceMat.name : materialName;
            foreach (char c in Path.GetInvalidFileNameChars()) safeName = safeName.Replace(c, '_');
            string path = $"{folder}/{safeName}.mat";

            Material existing = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (existing != null)
            {
                // 既存の.matを使い回す：中身を今回の変換結果で丸ごと上書き
                existing.shader = sourceMat.shader;
                existing.CopyPropertiesFromMaterial(sourceMat);
                EditorUtility.SetDirty(existing);
                return existing;
            }
            else
            {
                Material copy = new Material(sourceMat); // shaderとプロパティを丸ごと複製
                copy.name = safeName;
                AssetDatabase.CreateAsset(copy, path);
                return copy;
            }
        }

        // ★targetPrefab配下の全Rendererを走査し、oldMatを参照しているスロットをnewMatへ差し替える
        private void ReassignRendererMaterial(Material oldMat, Material newMat)
        {
            if (oldMat == newMat || newMat == null) return;

            var renderers = targetPrefab.GetComponentsInChildren<Renderer>(true);
            foreach (var r in renderers)
            {
                var mats = r.sharedMaterials;
                bool changed = false;
                for (int i = 0; i < mats.Length; i++)
                {
                    if (mats[i] == oldMat)
                    {
                        mats[i] = newMat;
                        changed = true;
                    }
                }
                if (changed)
                {
                    Undo.RecordObject(r, "Reassign lilToon Material");
                    r.sharedMaterials = mats;
                    EditorUtility.SetDirty(r);
                    UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(r.gameObject.scene);
                }
            }
        }

        private string GetSourceAssetPath()
        {
            if (targetPrefab == null) return null;
            string assetPath = "";
            GameObject prefabRoot = PrefabUtility.GetOutermostPrefabInstanceRoot(targetPrefab);
            if (prefabRoot != null)
            {
                var sourceAsset = PrefabUtility.GetCorrespondingObjectFromSource(prefabRoot);
                if (sourceAsset != null) assetPath = AssetDatabase.GetAssetPath(sourceAsset);
            }
            if (string.IsNullOrEmpty(assetPath))
            {
                var originalSource = PrefabUtility.GetCorrespondingObjectFromSource(targetPrefab);
                assetPath = originalSource != null ? AssetDatabase.GetAssetPath(originalSource) : AssetDatabase.GetAssetPath(targetPrefab);
            }
            return assetPath;
        }

        // 指定キーの配列にある各オブジェクトの"name"フィールドを、出現順のリストとして返す。
        // ParseNodeNamesと同じアルゴリズムだが、"nodes"以外の任意の配列にも使える汎用版。
        private List<string> ParseNamedArray(string json, string key)
        {
            var result = new List<string>();
            if (string.IsNullOrEmpty(json)) return result;

            string arr = FindJsonArrayObjects(json, key);
            if (string.IsNullOrEmpty(arr)) return result;

            int i = 1;
            int len = arr.Length;
            while (i < len)
            {
                while (i < len && (arr[i] == ',' || char.IsWhiteSpace(arr[i]))) i++;
                if (i >= len || arr[i] == ']') break;
                if (arr[i] != '{') { i++; continue; }

                int start = i;
                int depth = 0;
                bool inStr = false;
                for (; i < len; i++)
                {
                    char c = arr[i];
                    if (inStr)
                    {
                        if (c == '\\') { i++; continue; }
                        if (c == '"') inStr = false;
                        continue;
                    }
                    if (c == '"') { inStr = true; continue; }
                    if (c == '{') depth++;
                    else if (c == '}')
                    {
                        depth--;
                        if (depth == 0) { i++; break; }
                    }
                }
                string objText = arr.Substring(start, i - start);
                result.Add(ExtractStringField(objText, "name"));
            }
            return result;
        }

        // 簡易JSON文字列フィールド抽出（"key":"value" の value を取り出す。エスケープ対応）
        private string ExtractStringField(string obj, string key)
        {
            string pattern = "\"" + key + "\"";
            int idx = obj.IndexOf(pattern);
            if (idx == -1) return null;
            int colon = obj.IndexOf(':', idx + pattern.Length);
            if (colon == -1) return null;
            int q1 = obj.IndexOf('"', colon + 1);
            if (q1 == -1) return null;

            var sb = new System.Text.StringBuilder();
            int p = q1 + 1;
            while (p < obj.Length && obj[p] != '"')
            {
                if (obj[p] == '\\' && p + 1 < obj.Length) { sb.Append(obj[p + 1]); p += 2; continue; }
                sb.Append(obj[p]); p++;
            }
            return sb.ToString();
        }

        // デバッグ：物理ジョイント(オブジェクト型の配列)をConsoleに出力
        private void DumpJointsJson()
        {
            string json = GetRawJsonText();
            if (string.IsNullOrEmpty(json))
            {
                Debug.LogError("[MMD Debug] JSONを取得できませんでした。対象がGLBか確認してください。");
                return;
            }

            string jointsArr = FindJsonArrayObjects(json, "joints");
            if (!string.IsNullOrEmpty(jointsArr))
            {
                string preview = jointsArr.Length > 600 ? jointsArr.Substring(0, 600) + " ..." : jointsArr;
                Debug.Log("[MMD Debug] 物理ジョイント(先頭):\n" + preview);
                return;
            }

            Debug.LogWarning("[MMD Debug] オブジェクト型のjoints配列が見つかりません。物理ブロック周辺を出力します。");
            int anchor = json.IndexOf("\"physicsGltf\"");
            if (anchor == -1) anchor = json.IndexOf("\"rigidBodies\"");
            if (anchor == -1) anchor = json.IndexOf("\"rigid_bodies\"");
            if (anchor == -1) { Debug.LogError("[MMD Debug] 物理ブロックが見つかりません。"); return; }

            int len = Mathf.Min(800, json.Length - anchor);
            Debug.Log("[MMD Debug] 物理ブロック周辺:\n" + json.Substring(anchor, len));
        }

        // デバッグ：指定キーワードを名前に含む全ボーンの「現在のローカル回転」を一括出力する。
        private void DumpBoneRotationsByKeyword(string keyword)
        {
            Transform[] all = targetPrefab.GetComponentsInChildren<Transform>();
            int count = 0;
            foreach (var t in all)
            {
                if (!t.name.Contains(keyword)) continue;
                Vector3 e = t.localRotation.eulerAngles;
                Debug.Log($"[MMD Physics][ボーン一括確認] {t.name}: localRotation.euler=({e.x:F1}, {e.y:F1}, {e.z:F1})");
                count++;
            }
            Debug.Log($"[MMD Physics][ボーン一括確認] '{keyword}' を含むボーン {count} 件を出力しました。");
        }

        // デバッグ：JSON上の剛体データのmode値を一括確認する（0=Kinematic, 1=物理, 2=物理+位置合わせ）。
        // デバッグ：extras.mmd.materials（マテリアルのトゥーン/スフィア情報）の生JSONを確認する。
        //   仕様書のフィールド名(toon_texture_pipeline等)が実際のJSONと一致するとは
        //   限らない(rigidBodies/jointsでも実際は違った)ため、変換コードを書く前に必ず実データを見る。
        // デバッグ：glTFの"images"/"textures"配列の中身と、実際にUnity上へ
        //   サブアセットとしてインポートされているTexture2Dの一覧を突き合わせて表示する。
        //   sphereTexture/toonTexture の番号がどのテクスチャ実体に対応するのかを
        //   憶測せず確認するため。
        private void DumpTextureAssets()
        {
            string json = GetRawJsonText();
            if (string.IsNullOrEmpty(json))
            {
                Debug.LogError("[MMD Debug] JSONを取得できませんでした。");
                return;
            }

            // ① glTFの"images"配列（各画像のuri/name/bufferView等）
            string imagesArr = FindJsonArrayObjects(json, "images");
            if (!string.IsNullOrEmpty(imagesArr))
            {
                string preview = imagesArr.Length > 500 ? imagesArr.Substring(0, 500) + " ..." : imagesArr;
                Debug.Log("[MMD Debug] images配列(先頭):\n" + preview);
            }
            else Debug.LogWarning("[MMD Debug] images配列が見つかりません。");

            // ② glTFの"textures"配列（各テクスチャがどのimageを参照しているか）
            string texturesArr = FindJsonArrayObjects(json, "textures");
            if (!string.IsNullOrEmpty(texturesArr))
            {
                string preview = texturesArr.Length > 500 ? texturesArr.Substring(0, 500) + " ..." : texturesArr;
                Debug.Log("[MMD Debug] textures配列(先頭):\n" + preview);
            }
            else Debug.LogWarning("[MMD Debug] textures配列が見つかりません。");

            // ③ 実際にUnity側でこの.glbアセットの中にサブアセットとして
            //    インポートされているTexture2Dの一覧（名前・順序を実物で確認）
            string assetPath = "";
            GameObject prefabRoot = PrefabUtility.GetOutermostPrefabInstanceRoot(targetPrefab);
            if (prefabRoot != null)
            {
                var sourceAsset = PrefabUtility.GetCorrespondingObjectFromSource(prefabRoot);
                if (sourceAsset != null) assetPath = AssetDatabase.GetAssetPath(sourceAsset);
            }
            if (string.IsNullOrEmpty(assetPath))
            {
                var originalSource = PrefabUtility.GetCorrespondingObjectFromSource(targetPrefab);
                assetPath = originalSource != null ? AssetDatabase.GetAssetPath(originalSource) : AssetDatabase.GetAssetPath(targetPrefab);
            }

            if (string.IsNullOrEmpty(assetPath))
            {
                Debug.LogWarning("[MMD Debug] アセットパスが特定できませんでした。");
                return;
            }

            var allSubAssets = AssetDatabase.LoadAllAssetsAtPath(assetPath);
            int texCount = 0;
            foreach (var obj in allSubAssets)
            {
                if (obj is Texture2D tex)
                {
                    Debug.Log($"[MMD Debug][テクスチャ実体] {texCount}: name='{tex.name}' size={tex.width}x{tex.height}");
                    texCount++;
                }
            }
            Debug.Log($"[MMD Debug] このアセットには Texture2D サブアセットが {texCount} 件あります。");
        }

        private void DumpMaterialsJson()
        {
            string json = GetRawJsonText();
            if (string.IsNullOrEmpty(json))
            {
                Debug.LogError("[MMD Debug] JSONを取得できませんでした。対象がGLBか確認してください。");
                return;
            }

            // ①標準glTFの"materials"配列（pbrMetallicRoughness等）を確認
            string materialsArr = FindJsonArrayObjects(json, "materials");
            if (!string.IsNullOrEmpty(materialsArr))
            {
                string preview = materialsArr.Length > 700 ? materialsArr.Substring(0, 700) + " ..." : materialsArr;
                Debug.Log("[MMD Debug] materials配列(先頭):\n" + preview);
            }
            else
            {
                Debug.LogWarning("[MMD Debug] materials配列が見つかりません。");
            }

            // ②extras.mmd 以下にある、MMD固有のマテリアル拡張データを探す。
            //   "mmd" キーの周辺を広めに出力し、実際のフィールド名を目視確認する。
            int mmdIdx = json.IndexOf("\"mmd\"");
            if (mmdIdx != -1)
            {
                int len = Mathf.Min(1000, json.Length - mmdIdx);
                Debug.Log("[MMD Debug] \"mmd\"周辺(先頭1000文字):\n" + json.Substring(mmdIdx, len));
            }
            else
            {
                Debug.LogWarning("[MMD Debug] \"mmd\"キーが見つかりませんでした。");
            }

            // ③トゥーン/スフィア関連キーワードを含む箇所を個別に探す（キー名の実物確認用）
            string[] candidates = { "toon_texture", "toonTexture", "sphere_texture", "sphereTexture", "sphere_mode", "sphereMode" };
            foreach (var key in candidates)
            {
                int idx = json.IndexOf("\"" + key);
                if (idx != -1)
                {
                    int len = Mathf.Min(300, json.Length - idx);
                    Debug.Log($"[MMD Debug] キー '{key}' 発見(周辺300文字):\n" + json.Substring(idx, len));
                }
            }
        }

    }

    // ★マテリアル変換用データクラス（実際のJSON構造から確認したフィールド名を使用）
    [Serializable] internal class GltfMaterialListWrapper { public List<GltfMaterialData> list; }

    [Serializable]
    internal class GltfMaterialData
    {
        public string name;
        public GltfPbrData pbrMetallicRoughness;
        public bool doubleSided;
        public string alphaMode;   // "OPAQUE" / "MASK" / "BLEND"
        public float alphaCutoff = 0.5f;
        public GltfExtrasData extras;
    }

    [Serializable]
    internal class GltfPbrData
    {
        public float[] baseColorFactor;
        public float metallicFactor;
        public float roughnessFactor;
        public GltfTextureRef baseColorTexture;
    }

    [Serializable]
    internal class GltfTextureRef
    {
        public int index = -1;
    }

    [Serializable]
    internal class GltfExtrasData
    {
        public GltfMmdMaterialData mmd;
    }

    // extras.mmd の実際のフィールド名（キャメルケース。rigidBodies/jointsとは異なる）
    [Serializable]
    internal class GltfMmdMaterialData
    {
        public string nameEn;
        public float[] ambient;
        public float[] specular;
        public float specularPower;
        public int flags;
        public float[] edgeColor;
        public float edgeSize;
        public int sphereMode;    // 0=無効, 1=乗算(sph), 2=加算(spa), 3=サブテクスチャ(PMX標準仕様)
        public int sphereTexture = -1; // textures配列の番号
        public int toonTexture = -1;   // textures配列の番号（自前テクスチャ）
        public int toonShared = -1;    // MMD共有トゥーン番号(0始まり: 0=toon01.bmp〜9=toon10.bmp、-1=個別)。
                                       // プロジェクト内のtoon01〜10.bmpをファイル名検索して復元する
        public string memo;
        // ★エクスポーター拡張(元テクスチャ温存)対応：
        //   alphaClass: ベーステクスチャのα分類 ("opaque"/"mask"/"blend")
        //   origTexture: プリベイク前の無加工テクスチャのglTFテクスチャ番号(-1=なし)。
        //   旧GLBにはこれらのフィールドが無いが、JsonUtilityは未知フィールドを
        //   既定値のまま残すため後方互換(origTexture=-1で従来動作)。
        public string alphaClass;
        public int origTexture = -1;
    }

    // ★GLBバイナリから画像を直接抽出するためのデータクラス。
    //   UniGLTFは標準マテリアルのスロットから参照されていない画像（＝トゥーン/スフィア用）を
    //   Unityアセットとして作らないため、bufferViewの位置を辿って生バイト列を自前で取り出す。
    [Serializable] internal class GltfBufferViewListWrapper { public List<GltfBufferView> list; }
    [Serializable]
    internal class GltfBufferView
    {
        public int buffer;
        public int byteOffset;
        public int byteLength;
    }

    [Serializable] internal class GltfImageListWrapper { public List<GltfImageData> list; }
    [Serializable]
    internal class GltfImageData
    {
        public int bufferView = -1;
        public string mimeType;
        public string name;
        public string uri;
    }

    [Serializable] internal class GltfTextureListWrapper2 { public List<GltfTextureData2> list; }
    [Serializable]
    internal class GltfTextureData2
    {
        public int sampler;
        public int source = -1;
        public string name;
    }
}