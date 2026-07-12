using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

// ※ MmdPhysicsImportIndex は別ファイル(MmdPhysicsImportIndex.cs)に分離済み。
//   このファイルには絶対に再定義しないこと（missing参照の原因になる）。
//
// ★設計を全面刷新（mmd-for-unity を参考に、根本から作り直し）：
//   これまでは「物理剛体」を専用の別オブジェクトとして作り、揺れた結果を
//   毎フレームボーンへ「書き戻す」方式だった。これはボーンの子にすると
//   フィードバックループになる・書き戻しの基準計算が自己参照になる等、
//   本質的に不要な複雑さと不具合を生んでいた。
//
//   mmd-for-unity 方式では、Rigidbody と ConfigurableJoint を
//   「メッシュを実際に動かしているボーンTransformそのもの」に直接付与する。
//   コライダー形状だけは、位置・回転のオフセットを持たせるための子オブジェクトに
//   分離する。これにより、物理エンジンがボーンを直接動かし、メッシュは
//   スキニングにより自動的について来る。「書き戻し」という概念自体が不要になる。
namespace Mmd2GltfImporter
{
    public class MmdPhysicsImporterWindow : EditorWindow
    {
        [MenuItem("MMD/Physics Importer")]
        public static void ShowWindow()
        {
            GetWindow<MmdPhysicsImporterWindow>("MMD Physics");
        }

        private GameObject targetPrefab;

        // ★glTFの"nodes"配列から作る「番号→本当のノード名」の対応表。
        //   rbData.bone はこの配列の何番目かを指しているので、これが唯一
        //   信頼できる紐付け元になる（剛体名からの推測に頼らない）。
        private List<string> nodeNames = new List<string>();

        // ★剛体データの番号 → 実際にRigidbodyが付与されたボーンのRigidbody。
        //   ジョイント結合（ボタン2）で使う。ボタン1と2を同じセッションで
        //   連続実行すればこれで足りるが、コンパイル・ドメインリロードを挟むと
        //   このフィールドはリセットされるため、その場合はシーン上の
        //   MmdPhysicsImportIndex を走査して再構築する（RebuildIndexMapIfNeeded）。
        private Dictionary<int, Rigidbody> rigidBodyIndexToBoneRb = new Dictionary<int, Rigidbody>();

        // ★GLBバイナリから画像を直接抽出するための作業キャッシュ（マテリアル変換時のみ使用）
        private byte[] glbBytesCache;
        private List<GltfBufferView> bufferViewsCache;
        private List<GltfImageData> imagesCache;
        private List<GltfTextureData2> gltfTexturesCache;
        private int binChunkStart = -1;
        private Dictionary<int, Texture2D> extractedTextureCache = new Dictionary<int, Texture2D>();

        private void OnGUI()
        {
            GUILayout.Label("MMD glTF 統合インポーター (Rigidbody直付け版)", EditorStyles.boldLabel);
            EditorGUILayout.Space();

            targetPrefab = (GameObject)EditorGUILayout.ObjectField(
                "対象モデル (Scene上)", targetPrefab, typeof(GameObject), true);

            EditorGUILayout.Space();

            GUILayout.Label("【1】物理エンジンの構築", EditorStyles.miniBoldLabel);

            if (GUILayout.Button("1. 剛体とコライダーを配置 (古いエラーも自動掃除)") && targetPrefab != null)
            {
                GenerateRigidBodies();
            }

            if (GUILayout.Button("2. ジョイントを結合 (必ず1の直後に実行)") && targetPrefab != null)
            {
                ConnectJoints();
            }

            EditorGUILayout.Space();

            GUILayout.Label("【2】描画・見た目の修正", EditorStyles.miniBoldLabel);
            if (GUILayout.Button("3. マテリアルをlilToonへ変換（トゥーン/スフィア復元）") && targetPrefab != null)
            {
                ConvertMaterialsToLilToon();
            }

            EditorGUILayout.Space();

            GUILayout.Label("【デバッグ】", EditorStyles.miniBoldLabel);
            if (GUILayout.Button("ジョイントJSONを出力（キー名の確認用）") && targetPrefab != null)
            {
                DumpJointsJson();
            }

            if (GUILayout.Button("物理シーンを診断") && targetPrefab != null)
            {
                DiagnosePhysics();
            }

            if (GUILayout.Button("指定キーワードのボーン回転を一括出力（再生中/一時停止中も可）") && targetPrefab != null)
            {
                DumpBoneRotationsByKeyword("スカート");
            }

            if (GUILayout.Button("剛体JSONのmode値を一括確認（スカート）") && targetPrefab != null)
            {
                DumpRigidBodyModes("スカート");
            }

            if (GUILayout.Button("マテリアル(extras.mmd.materials)の生JSONを出力") && targetPrefab != null)
            {
                DumpMaterialsJson();
            }

            if (GUILayout.Button("テクスチャ実体とimages/texturesの対応を確認") && targetPrefab != null)
            {
                DumpTextureAssets();
            }
        }

        // ═══════════════════════════════════════════
        //  JSON 読み込み系
        // ═══════════════════════════════════════════
        private string GetRawJsonText()
        {
            if (targetPrefab == null) return null;

            string assetPath = "";
            GameObject prefabRoot = PrefabUtility.GetOutermostPrefabInstanceRoot(targetPrefab);
            if (prefabRoot != null)
            {
                UnityEngine.Object sourceAsset = PrefabUtility.GetCorrespondingObjectFromSource(prefabRoot);
                if (sourceAsset != null) assetPath = AssetDatabase.GetAssetPath(sourceAsset);
            }
            if (string.IsNullOrEmpty(assetPath))
            {
                var originalSource = PrefabUtility.GetCorrespondingObjectFromSource(targetPrefab);
                if (originalSource != null) assetPath = AssetDatabase.GetAssetPath(originalSource);
                else assetPath = AssetDatabase.GetAssetPath(targetPrefab);
            }

            if (string.IsNullOrEmpty(assetPath) || !assetPath.EndsWith(".glb", StringComparison.OrdinalIgnoreCase)) return null;

            try
            {
                byte[] bytes = File.ReadAllBytes(assetPath);
                if (bytes.Length < 20) return null;
                int chunkLength = BitConverter.ToInt32(bytes, 12);
                return Encoding.UTF8.GetString(bytes, 20, chunkLength);
            }
            catch { return null; }
        }

        // 最初に見つかった配列を返す（rigidBodies用）
        private string FindJsonArray(string json, string key)
        {
            int keyIndex = json.IndexOf("\"" + key + "\"");
            if (keyIndex == -1) return null;

            int startBracket = json.IndexOf("[", keyIndex);
            if (startBracket == -1) return null;

            int bracketCount = 1;
            for (int i = startBracket + 1; i < json.Length; i++)
            {
                if (json[i] == '[') bracketCount++;
                else if (json[i] == ']')
                {
                    bracketCount--;
                    if (bracketCount == 0) return json.Substring(startBracket, i - startBracket + 1);
                }
            }
            return null;
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

        private List<RigidBodyData> ExtractRigidBodies()
        {
            var bodiesList = new List<RigidBodyData>();
            string jsonText = GetRawJsonText();
            if (string.IsNullOrEmpty(jsonText)) return bodiesList;

            string rbArrayText = FindJsonArray(jsonText, "rigidBodies");
            if (string.IsNullOrEmpty(rbArrayText)) rbArrayText = FindJsonArray(jsonText, "rigid_bodies");

            if (!string.IsNullOrEmpty(rbArrayText))
            {
                var wrapper = JsonUtility.FromJson<RigidBodyListWrapper>("{\"list\":" + rbArrayText + "}");
                if (wrapper != null && wrapper.list != null) bodiesList = wrapper.list;
            }
            return bodiesList;
        }

        // ★joints 抽出：オブジェクト型の配列だけを狙う（skinの整数配列を回避）
        private List<JointData> ExtractJoints()
        {
            var list = new List<JointData>();
            string jsonText = GetRawJsonText();
            if (string.IsNullOrEmpty(jsonText)) return list;

            string arrText = FindJsonArrayObjects(jsonText, "joints");
            if (string.IsNullOrEmpty(arrText)) arrText = FindJsonArrayObjects(jsonText, "Joints");

            if (!string.IsNullOrEmpty(arrText))
            {
                var wrapper = JsonUtility.FromJson<JointListWrapper>("{\"list\":" + arrText + "}");
                if (wrapper != null && wrapper.list != null) list = wrapper.list;
            }
            return list;
        }

        // ═══════════════════════════════════════════
        //  剛体生成（★Rigidbody直付け版）
        //
        //  ・コライダー形状は「col_番号_名前」という子オブジェクトに持たせる
        //    （ボーンからの位置・回転オフセットのため）。
        //  ・Rigidbodyは実際のボーンTransformそのものに直接付与する。
        //    同じボーンに複数の剛体データが乗る場合（MMDの慣習）は、
        //    質量で加重平均して1つのRigidbodyにまとめる（mmd-for-unity と同じ方式）。
        // ═══════════════════════════════════════════
        private void GenerateRigidBodies()
        {
            var rigidBodies = ExtractRigidBodies();
            Transform[] allTransforms = targetPrefab.GetComponentsInChildren<Transform>();
            Undo.RegisterCompleteObjectUndo(targetPrefab, "Generate MMD Rigidbodies");

            // 物理剛体専用レイヤーを用意（自己衝突を切って暴れを防ぐ）
            int mmdLayer = EnsureLayer("MMDPhysics");
            if (mmdLayer != -1) SetLayerCollision(mmdLayer, mmdLayer, true); // 自己衝突を無効化（同士の絡まり暴れを防ぐ）

            // ★スカートだけ専用レイヤーに分離する。
            //   スカート同士は従来どおり衝突なし（同じ鎖内での絡まり暴れを防ぐ）が、
            //   スカート⇔体・脚（MMDPhysicsレイヤー）の衝突は復活させる。
            int skirtLayer = EnsureLayer("MMDPhysicsSkirt");
            if (skirtLayer != -1) SetLayerCollision(skirtLayer, skirtLayer, true);  // スカート同士は無効のまま
            if (skirtLayer != -1 && mmdLayer != -1) SetLayerCollision(skirtLayer, mmdLayer, false); // スカート⇔体は有効化

            // ★髪も同じ考え方で専用レイヤーに分離する。
            //   髪同士は無効のまま（絡まり暴れ防止）だが、髪⇔体・頭（MMDPhysics）の
            //   衝突は復活させる。これで髪が顔を突き抜けず、頭の丸みで受け止められる。
            int hairLayer = EnsureLayer("MMDPhysicsHair");
            if (hairLayer != -1) SetLayerCollision(hairLayer, hairLayer, true);
            if (hairLayer != -1 && mmdLayer != -1) SetLayerCollision(hairLayer, mmdLayer, false);

            // 古い物理オブジェクト(rb_/col_から始まるもの、旧方式のMMD_PhysicsRig)を根こそぎ完全消去
            foreach (var t in allTransforms)
            {
                if (t != null && (t.name.StartsWith("rb_") || t.name.StartsWith("col_") || t.name == "MMD_PhysicsRig"))
                {
                    DestroyImmediate(t.gameObject);
                }
            }

            // 旧・新問わず、以前の実行で付与されたRigidbody/ConfigurableJointも一旦全部外して作り直す
            foreach (var oldJoint in targetPrefab.GetComponentsInChildren<ConfigurableJoint>())
                DestroyImmediate(oldJoint);
            foreach (var oldRb in targetPrefab.GetComponentsInChildren<Rigidbody>())
                DestroyImmediate(oldRb);

            allTransforms = targetPrefab.GetComponentsInChildren<Transform>();

            // JSONのnodes配列から「番号→本当のノード名」を先に構築しておく
            nodeNames = ParseNodeNames(GetRawJsonText());
            Debug.Log($"[MMD Physics] nodes配列を{nodeNames.Count}件解析しました。");

            float scaleFactor = EstimateMmdScale(rigidBodies, allTransforms);
            Debug.Log($"[MMD Physics] MMD→Unity 推定スケール: {scaleFactor:F4}");

            rigidBodyIndexToBoneRb = new Dictionary<int, Rigidbody>();
            int createdCount = 0;

            for (int i = 0; i < rigidBodies.Count; i++)
            {
                RigidBodyData rbData = rigidBodies[i];
                Transform boneTransform = FindBoneByGltfIndexOrName(allTransforms, rbData.bone, rbData.name);
                if (boneTransform == null) continue;

                // ★pos/rot をJSONから正しく読み、コライダー子オブジェクトをその位置・傾きで配置する。
                //   pos: モデル原点基準のワールド座標（MMD原寸）→ scaleFactor で換算。
                //   rot: ワールド回転のオイラー角（ラジアン、既にglTF軸に変換済み）→ そのままdeg変換。
                //   posはモデル自身のローカル原点基準の座標なので、targetPrefab.transform
                //   （モデルのルート変換）を通して初めて正しいワールド座標になる。
                Vector3 worldPos = boneTransform.position;
                if (rbData.pos != null && rbData.pos.Count >= 3)
                {
                    Vector3 localPos = new Vector3(rbData.pos[0], rbData.pos[1], rbData.pos[2]) * scaleFactor;
                    worldPos = targetPrefab.transform.TransformPoint(localPos);
                }

                Quaternion worldRot = boneTransform.rotation;
                if (rbData.rot != null && rbData.rot.Count >= 3)
                {
                    Quaternion localRot = Quaternion.Euler(
                        rbData.rot[0] * Mathf.Rad2Deg,
                        rbData.rot[1] * Mathf.Rad2Deg,
                        rbData.rot[2] * Mathf.Rad2Deg);
                    worldRot = targetPrefab.transform.rotation * localRot;
                }

                bool isSkirt = rbData.name != null && rbData.name.Contains("スカート");
                bool isHair = rbData.name != null && (rbData.name.Contains("髪") || rbData.name.Contains("ツインテ"));

                // ── コライダー形状は子オブジェクトに（ボーンからの位置・回転オフセット）──
                GameObject colObj = new GameObject($"col_{i}_{rbData.name}");
                colObj.transform.position = worldPos;
                colObj.transform.rotation = worldRot;
                colObj.transform.SetParent(boneTransform, true); // worldPositionStays=true でこの姿勢を保持

                int layerToUse = mmdLayer;
                if (isSkirt && skirtLayer != -1) layerToUse = skirtLayer;
                else if (isHair && hairLayer != -1) layerToUse = hairLayer;
                if (layerToUse != -1) colObj.layer = layerToUse;

                // スカートは脚との衝突を有効にした関係で、恒常的な深いめり込みを避けるため
                // コライダーサイズを少し縮める。
                // スカートは体との衝突を有効にしたため、単純形状同士の重なり(めり込み)を
                // 減らすべく縮小率を強める(0.8→0.6)。髪も同様に少し縮めて突き抜けにくくする。
                float colliderScale = scaleFactor;
                if (isSkirt) colliderScale = scaleFactor * 0.6f;
                else if (isHair) colliderScale = scaleFactor * 0.85f;
                AttachCollider(colObj, rbData, colliderScale);

                var idxComp = colObj.AddComponent<MmdPhysicsImportIndex>();
                idxComp.absoluteDataIndex = i;
                idxComp.boneName = boneTransform.name;

                // ── Rigidbodyはボーン自身に直接付与（mmd-for-unity 方式）──
                //    同じボーンに複数の剛体データが乗る場合は質量で加重平均して1つにまとめる。
                bool isKinematicBody = (rbData.mode == 0);
                Rigidbody rb = boneTransform.GetComponent<Rigidbody>();
                if (rb == null)
                {
                    rb = boneTransform.gameObject.AddComponent<Rigidbody>();
                    rb.mass = Mathf.Max(rbData.mass, 0.01f);
                    rb.linearDamping = Mathf.Max(rbData.linear_damping, 0.05f);
                    rb.angularDamping = Mathf.Max(rbData.angular_damping, 0.2f);
                    rb.isKinematic = isKinematicBody;
                }
                else
                {
                    float newMass = Mathf.Max(rbData.mass, 0.01f);
                    float totalMass = rb.mass + newMass;
                    float newLinDamp = Mathf.Max(rbData.linear_damping, 0.05f);
                    float newAngDamp = Mathf.Max(rbData.angular_damping, 0.2f);
                    rb.linearDamping = (rb.linearDamping * rb.mass + newLinDamp * newMass) / totalMass;
                    rb.angularDamping = (rb.angularDamping * rb.mass + newAngDamp * newMass) / totalMass;
                    rb.mass = totalMass;
                    if (!isKinematicBody) rb.isKinematic = false; // どれか1つでも物理対象なら物理を優先
                }
                if (isSkirt || isHair) rb.maxDepenetrationVelocity = 1f; // 衝突再有効化に伴う初期めり込みの急激な弾けを防ぐ

                rigidBodyIndexToBoneRb[i] = rb;
                createdCount++;
            }

            Debug.Log($"[MMD Physics] 剛体 {createdCount} 個を生成しました（Rigidbodyはボーン本体に直接付与）。");
        }

        // ═══════════════════════════════════════════
        //  ジョイント結合（データ駆動・実キー名対応・Rigidbody直付け版）
        // ═══════════════════════════════════════════
        private void ConnectJoints()
        {
            RebuildIndexMapIfNeeded();

            if (rigidBodyIndexToBoneRb == null || rigidBodyIndexToBoneRb.Count == 0)
            {
                Debug.LogError("[MMD Physics] 剛体が見つかりません。先にボタン1を実行してください。");
                return;
            }

            Undo.RegisterCompleteObjectUndo(targetPrefab, "Connect MMD Joints (Data Driven)");

            // 既存の ConfigurableJoint を全消去（再実行で入れ替え可能に）
            foreach (var j in targetPrefab.GetComponentsInChildren<ConfigurableJoint>())
                DestroyImmediate(j);

            var joints = ExtractJoints();

            if (joints != null && joints.Count > 0)
            {
                int connected = ConnectJointsFromData(joints, rigidBodyIndexToBoneRb);
                Debug.Log($"[MMD Physics] ジョイント {connected} 個をデータ駆動で結合しました！（joints総数 {joints.Count}）");
            }
            else
            {
                Debug.LogWarning("[MMD Physics] joints データが見つからないため接続をスキップしました。");
            }
        }

        // ドメインリロード等で rigidBodyIndexToBoneRb がリセットされた場合、
        // シーン上の MmdPhysicsImportIndex（コライダー子オブジェクトに付いている）を
        // 走査して再構築する。absoluteDataIndex → その親ボーンのRigidbody、という対応。
        private void RebuildIndexMapIfNeeded()
        {
            if (rigidBodyIndexToBoneRb != null && rigidBodyIndexToBoneRb.Count > 0) return;
            if (targetPrefab == null) return;

            rigidBodyIndexToBoneRb = new Dictionary<int, Rigidbody>();
            var indices = targetPrefab.GetComponentsInChildren<MmdPhysicsImportIndex>();
            foreach (var idx in indices)
            {
                if (idx.transform.parent == null) continue;
                var rb = idx.transform.parent.GetComponent<Rigidbody>();
                if (rb == null) continue;
                if (!rigidBodyIndexToBoneRb.ContainsKey(idx.absoluteDataIndex))
                    rigidBodyIndexToBoneRb[idx.absoluteDataIndex] = rb;
            }

            if (rigidBodyIndexToBoneRb.Count > 0)
                Debug.Log($"[MMD Physics] インデックス対応表をシーンから再構築しました（{rigidBodyIndexToBoneRb.Count}件）。");
        }

        // ★データ駆動の本体（rigid_a / rigid_b / rot_min / rot_max / spring_rot ...）
        private int ConnectJointsFromData(List<JointData> joints, Dictionary<int, Rigidbody> rbMap)
        {
            int connected = 0;
            int skipped = 0;

            foreach (var jd in joints)
            {
                Rigidbody parentRb, childRb;
                if (!rbMap.TryGetValue(jd.rigid_a, out parentRb)) { skipped++; continue; }
                if (!rbMap.TryGetValue(jd.rigid_b, out childRb)) { skipped++; continue; }
                if (parentRb == null || childRb == null || parentRb == childRb) { skipped++; continue; }

                ConfigurableJoint joint = childRb.gameObject.AddComponent<ConfigurableJoint>();
                joint.connectedBody = parentRb;
                joint.enableCollision = false; // 接続相手とは衝突させない

                // 初期ズレ防止
                joint.axis = Vector3.right;
                joint.secondaryAxis = Vector3.up;
                joint.configuredInWorldSpace = false;
                joint.autoConfigureConnectedAnchor = true;

                // ── 直線可動（min==max==0 でロック）──
                joint.xMotion = MotionFor(jd.pos_min, jd.pos_max, 0);
                joint.yMotion = MotionFor(jd.pos_min, jd.pos_max, 1);
                joint.zMotion = MotionFor(jd.pos_min, jd.pos_max, 2);

                float maxLin = Mathf.Max(
                    Mathf.Abs(SafeGet(jd.pos_max, 0, 0f)),
                    Mathf.Abs(SafeGet(jd.pos_max, 1, 0f)),
                    Mathf.Abs(SafeGet(jd.pos_max, 2, 0f)));
                if (maxLin > 0f)
                {
                    var lin = joint.linearLimit; lin.limit = maxLin; joint.linearLimit = lin;
                }

                // ── 回転可動（ラジアン→度、左右非対称対応）──
                joint.angularXMotion = MotionFor(jd.rot_min, jd.rot_max, 0);
                joint.angularYMotion = MotionFor(jd.rot_min, jd.rot_max, 1);
                joint.angularZMotion = MotionFor(jd.rot_min, jd.rot_max, 2);

                // ★MMDの角度制限は「直立の基本ポーズ」基準に作られており、かなり狭い。
                //   ダンス等で脚や腰の向きが基本ポーズと大きく変わると、重力で垂れたい
                //   スカート等がこの狭い壁に押し付けられて固まることがあるため、
                //   少し「遊び」を追加して吸収できるようにする。
                const float angularSlackDeg = 45f;

                var hx = joint.highAngularXLimit;
                hx.limit = SafeGet(jd.rot_max, 0, 0f) * Mathf.Rad2Deg + angularSlackDeg;
                joint.highAngularXLimit = hx;

                var lx = joint.lowAngularXLimit;
                lx.limit = SafeGet(jd.rot_min, 0, 0f) * Mathf.Rad2Deg - angularSlackDeg;
                joint.lowAngularXLimit = lx;

                var ay = joint.angularYLimit;
                ay.limit = Mathf.Max(Mathf.Abs(SafeGet(jd.rot_min, 1, 0f)), Mathf.Abs(SafeGet(jd.rot_max, 1, 0f))) * Mathf.Rad2Deg + angularSlackDeg;
                joint.angularYLimit = ay;

                var az = joint.angularZLimit;
                az.limit = Mathf.Max(Mathf.Abs(SafeGet(jd.rot_min, 2, 0f)), Mathf.Abs(SafeGet(jd.rot_max, 2, 0f))) * Mathf.Rad2Deg + angularSlackDeg;
                joint.angularZLimit = az;

                // ── ばね（spring_rot → 回転ドライブ）──
                ApplyRotationSpring(joint, jd.spring_rot);

                connected++;
            }

            if (skipped > 0)
                Debug.LogWarning($"[MMD Physics] {skipped} 件のジョイントは対応する剛体が見つからずスキップしました。");

            return connected;
        }

        private void ApplyRotationSpring(ConfigurableJoint joint, List<float> springRot)
        {
            float sx = SafeGet(springRot, 0, 0f);
            float syz = Mathf.Max(SafeGet(springRot, 1, 0f), SafeGet(springRot, 2, 0f));

            sx = Mathf.Max(sx, 3f);
            syz = Mathf.Max(syz, 3f);

            joint.rotationDriveMode = RotationDriveMode.XYAndZ;

            var dx = joint.angularXDrive;
            dx.positionSpring = sx;
            dx.positionDamper = Mathf.Max(sx * 0.1f, 0.5f);
            dx.maximumForce = Mathf.Infinity;
            joint.angularXDrive = dx;

            var dyz = joint.angularYZDrive;
            dyz.positionSpring = syz;
            dyz.positionDamper = Mathf.Max(syz * 0.1f, 0.5f);
            dyz.maximumForce = Mathf.Infinity;
            joint.angularYZDrive = dyz;
        }

        private ConfigurableJointMotion MotionFor(List<float> min, List<float> max, int axis)
        {
            float lo = SafeGet(min, axis, 0f);
            float hi = SafeGet(max, axis, 0f);
            if (Mathf.Approximately(lo, 0f) && Mathf.Approximately(hi, 0f))
                return ConfigurableJointMotion.Locked;
            return ConfigurableJointMotion.Limited;
        }

        private float SafeGet(List<float> list, int i, float def)
        {
            if (list != null && i >= 0 && i < list.Count) return list[i];
            return def;
        }

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
                if (existing != null) return existing;
                return ExtractTextureFromGlb(texIndex, assetPath);
            }

            Shader lilShader = Shader.Find("lilToon");
            if (lilShader == null)
            {
                Debug.LogError("[MMD Material] lilToonシェーダーが見つかりません。lilToonをプロジェクトに導入してください。");
                return;
            }

            int converted = 0, skipped = 0, sphereApplied = 0, toonApplied = 0;

            foreach (var md in materialList)
            {
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
                bool isCutout = (modeInt == 1);

                // シェーダー未設定(初回)ならまずlilToonを割り当ててから正規セットアップを呼ぶ
                if (mat.shader != lilShader) mat.shader = lilShader;
                SetupLilToonRenderingMode(mat, modeInt, 0); // transparentMode=0(Normal)

                // ── ベースカラー ──
                if (md.pbrMetallicRoughness != null)
                {
                    var c = md.pbrMetallicRoughness.baseColorFactor;
                    if (c != null && c.Length >= 4) mat.SetColor("_Color", new Color(c[0], c[1], c[2], c[3]));

                    if (md.pbrMetallicRoughness.baseColorTexture != null)
                    {
                        var tex = FindTextureByIndex(md.pbrMetallicRoughness.baseColorTexture.index);
                        if (tex != null) mat.SetTexture("_MainTex", tex);
                    }
                }

                // ── アルファ閾値（MASK時）──
                if (isCutout) mat.SetFloat("_Cutoff", md.alphaCutoff > 0 ? md.alphaCutoff : 0.5f);

                // ── 両面描画 ──
                mat.SetFloat("_Cull", md.doubleSided ? 0f : 2f); // 0=Off(両面), 2=Back(片面)

                // ── MMD固有：トゥーンテクスチャ・スフィアマップ ──
                var mmd = md.extras != null ? md.extras.mmd : null;

                // ★診断ログ：mmdデータそのものがnullなのか、値は読めているのに
                //   テクスチャ照合で失敗しているのかを切り分けるため。
                if (mmd == null)
                {
                    Debug.LogWarning($"[MMD Material][診断] '{md.name}': extras.mmd の解析結果がnullです。");
                }
                else
                {
                    Debug.Log($"[MMD Material][診断] '{md.name}': sphereMode={mmd.sphereMode}, sphereTexture={mmd.sphereTexture}, toonTexture={mmd.toonTexture}, toonShared={mmd.toonShared}");
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
                        // MMD共有トゥーン(toon00〜10.bmp)は今回のglTFに画像として
                        // 埋め込まれていないため復元できない。ログのみ残す。
                        Debug.LogWarning($"[MMD Material] '{md.name}' は共有トゥーン(toonShared={mmd.toonShared})を使用しており、復元できません。");
                    }

                    if (mmd.sphereTexture >= 0 && mmd.sphereMode > 0)
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
                converted++;
            }

            AssetDatabase.SaveAssets();
            Debug.Log($"[MMD Material] {converted}件をlilToonへ変換しました（トゥーン:{toonApplied}件, スフィア:{sphereApplied}件, スキップ:{skipped}件）。");
        }

        // モデルの元アセットパス(.glb)を取得する共通処理
        // ★リフレクションで lilToon.lilToonInspector.SetupMaterialWithRenderingMode を呼ぶ。
        //   lilToon.Editor アセンブリは "autoReferenced": false のため、通常の
        //   using lilToon; では型が見えない（アセンブリ定義ファイルの追加が必要になる）。
        //   それを避けるため、型名を文字列で指定するリフレクションで直接呼び出す。
        //   renderingMode: 0=Opaque, 1=Cutout, 2=Transparent
        //   transparentMode: 0=Normal
        private void SetupLilToonRenderingMode(Material mat, int renderingMode, int transparentMode)
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
                new System.Type[] { typeof(Material), renderingModeType, transparentModeType },
                null);

            if (method == null)
            {
                Debug.LogError("[MMD Material] SetupMaterialWithRenderingMode メソッドが見つかりませんでした（lilToonのバージョン差異の可能性）。");
                return;
            }

            method.Invoke(null, new object[] { mat, renderingModeVal, transparentModeVal });
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

        private void FixMaterials()
        {
            Renderer[] renderers = targetPrefab.GetComponentsInChildren<Renderer>();
            Undo.RegisterCompleteObjectUndo(targetPrefab, "Fix MMD Materials Premium");

            int fixedCount = 0;
            foreach (var renderer in renderers)
            {
                Material[] mats = renderer.sharedMaterials;
                for (int i = 0; i < mats.Length; i++)
                {
                    if (mats[i] == null) continue;
                    Material mat = mats[i];
                    Shader unlitShader = Shader.Find("Universal Render Pipeline/Unlit");
                    if (unlitShader != null && mat.shader != unlitShader) mat.shader = unlitShader;

                    if (mat.HasProperty("_Cull")) mat.SetFloat("_Cull", 0f);
                    mat.SetOverrideTag("RenderType", "Opaque");
                    mat.SetOverrideTag("Queue", "Geometry");

                    if (mat.HasProperty("_AlphaClip")) mat.SetFloat("_AlphaClip", 1f);
                    if (mat.HasProperty("_Cutoff")) mat.SetFloat("_Cutoff", 0.5f);
                    if (mat.HasProperty("_ZWrite")) mat.SetFloat("_ZWrite", 1f);

                    mat.renderQueue = 2000;
                    mat.EnableKeyword("_ALPHATEST_ON");
                    fixedCount++;
                }
                renderer.sharedMaterials = mats;
            }
            AssetDatabase.SaveAssets();
            Debug.Log($"[MMD Material] マテリアルを修正しました。");
        }

        // ═══════════════════════════════════════════
        //  補助メソッド ＋ デバッグ出力
        // ═══════════════════════════════════════════
        // shape: 0=球, 1=箱, 2=カプセル。size は MMD原寸なので scale で換算する。
        private void AttachCollider(GameObject obj, RigidBodyData rbData, float scale)
        {
            if (rbData.size == null || rbData.size.Count == 0) return;
            switch (rbData.shape)
            {
                case 0: // 球
                    obj.AddComponent<SphereCollider>().radius = rbData.size[0] * scale;
                    break;
                case 1: // 箱（size は半寸法 → 全寸法へ）
                    obj.AddComponent<BoxCollider>().size = new Vector3(
                        rbData.size[0] * 2 * scale, rbData.size[1] * 2 * scale, rbData.size[2] * 2 * scale);
                    break;
                case 2: // カプセル
                    var cap = obj.AddComponent<CapsuleCollider>();
                    cap.radius = rbData.size[0] * scale;
                    cap.height = (rbData.size[1] + rbData.size[0] * 2f) * scale;
                    cap.direction = 1;
                    break;
                default:
                    obj.AddComponent<SphereCollider>().radius = Mathf.Max(rbData.size[0] * scale, 0.01f);
                    break;
            }
        }

        // MMD原寸→Unityメートルのスケールを、ボーン追従剛体(mode==0)のボーン位置から推定する。
        private float EstimateMmdScale(List<RigidBodyData> bodies, Transform[] transforms)
        {
            if (bodies == null || bodies.Count == 0) return 0.08f;

            List<float> ratios = new List<float>();
            foreach (var d in bodies)
            {
                if (d.pos == null || d.pos.Count < 3) continue;
                Transform bone = FindBoneByGltfIndexOrName(transforms, d.bone, d.name);
                if (bone == null) continue;

                float mmdY = d.pos[1];
                float localY = targetPrefab.transform.InverseTransformPoint(bone.position).y;
                if (Mathf.Abs(mmdY) > 1f && Mathf.Abs(localY) > 0.01f)
                    ratios.Add(localY / mmdY);
            }

            if (ratios.Count == 0) return 0.08f;

            ratios.Sort();
            float median = ratios[ratios.Count / 2];
            if (median <= 0.0001f || median > 1f) return 0.08f;
            return median;
        }

        // 指定レイヤー同士の衝突を有効/無効切り替え、設定ファイルにも永続化する。
        // ignore=true で「衝突しない」、ignore=false で「衝突する」。
        private void SetLayerCollision(int layerA, int layerB, bool ignore)
        {
            Physics.IgnoreLayerCollision(layerA, layerB, ignore);

            var assets = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/DynamicsManager.asset");
            if (assets == null || assets.Length == 0)
            {
                Debug.LogWarning("[MMD Physics] DynamicsManagerが読めず、マトリクスを永続化できませんでした。" +
                                 " 実行時には設定されています。");
                return;
            }

            SerializedObject dm = new SerializedObject(assets[0]);
            SerializedProperty matrix = dm.FindProperty("m_LayerCollisionMatrix");
            if (matrix == null || !matrix.isArray || layerA >= matrix.arraySize || layerB >= matrix.arraySize)
            {
                Debug.LogWarning("[MMD Physics] 衝突マトリクスの書き込みに失敗しました。実行時のみ有効です。");
                return;
            }

            void SetBit(int row, int col)
            {
                SerializedProperty rowProp = matrix.GetArrayElementAtIndex(row);
                long mask = rowProp.longValue;
                if (ignore) mask &= ~(1L << col); else mask |= (1L << col);
                rowProp.longValue = mask;
            }

            SetBit(layerA, layerB);
            SetBit(layerB, layerA);
            dm.ApplyModifiedProperties();

            Debug.Log($"[MMD Physics] レイヤー{layerA}⇔{layerB} の衝突を{(ignore ? "無効化" : "有効化")}しました。");
        }

        // 指定名のレイヤーが無ければ空きスロット(8〜31)に自動作成して番号を返す
        private int EnsureLayer(string layerName)
        {
            int existing = LayerMask.NameToLayer(layerName);
            if (existing != -1) return existing;

            var assets = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset");
            if (assets == null || assets.Length == 0)
            {
                Debug.LogWarning("[MMD Physics] TagManagerが読めず、レイヤーを作成できませんでした。");
                return -1;
            }

            SerializedObject tagManager = new SerializedObject(assets[0]);
            SerializedProperty layers = tagManager.FindProperty("layers");

            for (int i = 8; i < 32; i++)
            {
                SerializedProperty sp = layers.GetArrayElementAtIndex(i);
                if (sp != null && string.IsNullOrEmpty(sp.stringValue))
                {
                    sp.stringValue = layerName;
                    tagManager.ApplyModifiedProperties();
                    Debug.Log($"[MMD Physics] レイヤー '{layerName}' を作成しました (index {i})。");
                    return i;
                }
            }
            Debug.LogWarning($"[MMD Physics] 空きレイヤーが無く '{layerName}' を作成できませんでした。");
            return -1;
        }

        // MMDは「同じボーンに複数の物理形状を積む」場合、剛体名を"頭2"のように
        // ボーン名+連番にする慣習があるが、実際のボーンには番号が付かない("頭"のみ)。
        // そのため末尾の数字を取り除いた名前でも照合できるようにする。
        private string StripTrailingDigits(string s)
        {
            int end = s.Length;
            while (end > 0 && char.IsDigit(s[end - 1])) end--;
            return s.Substring(0, end);
        }

        private Transform FindBoneByGltfIndexOrName(Transform[] transforms, int index, string name)
        {
            // ①最優先：同じJSONのnodes配列から「この番号の本当のノード名」を取得し、
            //   それで完全一致検索する。剛体名(例："胸部")は対応ボーン名("上半身2")と
            //   一致するとは限らないため、これが唯一の確実なルート。
            if (nodeNames != null && index >= 0 && index < nodeNames.Count)
            {
                string trueName = nodeNames[index];
                if (!string.IsNullOrEmpty(trueName))
                {
                    foreach (var t in transforms) if (t.name == trueName) return t;
                    foreach (var t in transforms) if (t.name.Contains(trueName)) return t;
                }
            }

            // ②次善：剛体自身の名前で照合（従来のヒューリスティック。連番除去つき）
            if (!string.IsNullOrEmpty(name))
            {
                string stripped = StripTrailingDigits(name);
                bool hasStripped = stripped.Length > 0 && stripped != name;

                foreach (var t in transforms) if (t.name == name) return t;
                if (hasStripped)
                    foreach (var t in transforms) if (t.name == stripped) return t;
                foreach (var t in transforms) if (t.name.Contains(name)) return t;
                if (hasStripped)
                    foreach (var t in transforms) if (t.name.Contains(stripped)) return t;
            }

            // ③最後の保険：番号をUnityの並び順にそのまま当てる（不確実）
            if (index >= 0 && index < transforms.Length)
            {
                Transform fallback = transforms[index];
                Debug.LogWarning($"[MMD Physics][ボーン照合失敗] 剛体名 '{name}' に一致するボーンが見つからず、" +
                                  $"番号{index}のボーン '{fallback.name}' を代用しました。位置がズレている可能性があります。");
                return fallback;
            }
            return null;
        }

        // ★glTFの"nodes"配列を解析し、「番号→ノード名」の一覧を返す。
        private List<string> ParseNodeNames(string json)
        {
            var result = new List<string>();
            if (string.IsNullOrEmpty(json)) return result;

            string arr = FindJsonArrayObjects(json, "nodes");
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

        // デバッグ：物理シーン全体の土台を数値で確認する
        private void DiagnosePhysics()
        {
            {
                string j = GetRawJsonText();
                if (!string.IsNullOrEmpty(j))
                {
                    string arr = FindJsonArrayObjects(j, "rigidBodies");
                    if (string.IsNullOrEmpty(arr)) arr = FindJsonArrayObjects(j, "rigid_bodies");
                    if (!string.IsNullOrEmpty(arr))
                    {
                        string preview = arr.Length > 500 ? arr.Substring(0, 500) + " ..." : arr;
                        Debug.Log("[剛体生JSON] " + preview);
                    }
                    else Debug.LogWarning("[剛体生JSON] rigidBodies配列が見つかりません");
                }
            }

            Renderer[] rends = targetPrefab.GetComponentsInChildren<Renderer>();
            if (rends.Length > 0)
            {
                Bounds b = rends[0].bounds;
                foreach (var r in rends) b.Encapsulate(r.bounds);
                Debug.Log($"[診断] モデル全高: {b.size.y:F2} (幅 {b.size.x:F2})　※1前後が理想、20前後ならMMD原寸で巨大");
            }

            Debug.Log($"[診断] ルートscale: {targetPrefab.transform.lossyScale}");
            Debug.Log($"[診断] Gravity: {Physics.gravity}");

            Rigidbody[] rbs = targetPrefab.GetComponentsInChildren<Rigidbody>();
            int kinematic = 0;
            foreach (var rb in rbs) if (rb.isKinematic) kinematic++;
            Debug.Log($"[診断] 剛体数: {rbs.Length}　うちKinematic(動かない): {kinematic}　動く剛体: {rbs.Length - kinematic}");

            ConfigurableJoint[] joints = targetPrefab.GetComponentsInChildren<ConfigurableJoint>();
            Debug.Log($"[診断] ジョイント数: {joints.Length}");
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

        private void DumpRigidBodyModes(string keyword)
        {
            var bodies = ExtractRigidBodies();
            int count = 0;
            foreach (var d in bodies)
            {
                if (d.name == null || !d.name.Contains(keyword)) continue;
                Debug.Log($"[MMD Physics][mode確認] {d.name}: mode={d.mode}");
                count++;
            }
            Debug.Log($"[MMD Physics][mode確認] '{keyword}' を含む剛体 {count} 件のmodeを出力しました。");
        }
    }

    [Serializable] internal class RigidBodyListWrapper { public List<RigidBodyData> list; }
    [Serializable] internal class JointListWrapper { public List<JointData> list; }

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
        public int sphereMode;    // 0=無効, 1=加算(仕様書記載), 2=乗算(仕様書記載)
        public int sphereTexture = -1; // textures配列の番号
        public int toonTexture = -1;   // textures配列の番号（自前テクスチャ）
        public int toonShared = -1;    // MMD共有トゥーン(toon00〜10)の番号。今回は復元非対応
        public string memo;
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
