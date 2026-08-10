// ===========================================================================
// ゲーム内の当たり判定（ヒットボックス）を PMX の剛体定義から生成する。
//
// なぜ別物なのか:
//   かつてのPhysX経路は「剛体+ジョイントで揺らす」ためにコライダーを作っていたが、
//   物理は自作Bulletエンジンへ一本化して撤去した。しかしゲーム側の当たり判定には
//   Unity のコライダーが要る（エンジン内部の CollisionShape は Physics.Raycast や
//   OnTriggerEnter からは見えない）。そこで「判定専用」として作り直したのが本ファイル。
//
// 設計:
//   - Rigidbody は付けない。コライダーをボーンの子に置くだけなので、アニメでも
//     自作エンジンでもボーンが動けば自動で追従する。物理計算は一切増えない。
//   - 既定は isTrigger=true。相手側（弾・剣など）の Rigidbody で判定が飛ぶ、
//     Unity で最も一般的なヒットボックス構成。
//   - ★Rigidbody を付けない理由はもう一つある: MmdPhysicsBackendSwitch は Custom 時に
//     配下の全 Rigidbody へ detectCollisions=false を設定するため、Rigidbody を持つ
//     ヒットボックスは黙って無効化されてしまう。コライダーのみなら原理的に踏まない。
//
// 座標:
//   剛体もボーンも GlbPhysicsReader が読む raw PMX 値。バインド時のボーンは回転恒等・
//   位置のみなので、ボーンからのオフセットは PmxPhysicsBuilder.ComputeOffset と同じ式
//   (boneWorld.InverseTimes(bodyWorld)) で求まる。Unityへは MmdPhysicsBehaviour と同じ
//   「単位スケールのみ・Z反転なし」で写す。オイラー順序も engine の Quat.FromEuler に
//   合わせる（Unity の Quaternion.Euler とは順序が違うので使ってはいけない）。
// ===========================================================================
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;
using BulletPhysics;
using BulletPhysics.Pmx;

namespace Mmd2GltfImporter
{
    public static class MmdHitboxBuilder
    {
        /// <summary>生成対象の絞り込み。PMXの物理モードで分ける。</summary>
        public enum Scope
        {
            BodyOnly,    // mode 0 = ボーン追従。頭・胴・手足＝ヒットボックス向き
            SwayOnly,    // mode 1,2 = 物理演算。髪・スカート
            All,
        }

        public static readonly string[] ScopeLabelsJa =
            { "体パーツのみ (mode 0・推奨)", "髪・スカートのみ (mode 1,2)", "すべて" };
        public static readonly string[] ScopeLabelsEn =
            { "Body parts only (mode 0, recommended)", "Hair & skirt only (mode 1,2)", "All" };

        private const string RootName = "MMD_Hitboxes";

        /// <summary>PMXの物理モードが対象範囲に含まれるか。</summary>
        private static bool InScope(int mode, Scope scope) => scope switch
        {
            Scope.BodyOnly => mode == 0,
            Scope.SwayOnly => mode != 0,
            _ => true,
        };

        // ─────────────────────────────────────────────
        //  生成
        // ─────────────────────────────────────────────
        public static void Build(GameObject modelRoot, string glbAbsPath, Scope scope,
                                 string layerName, bool asTrigger)
        {
            if (modelRoot == null || string.IsNullOrEmpty(glbAbsPath)) return;

            PmxPhysicsModel model;
            float unitScale;
            try
            {
                model = GlbPhysicsReader.LoadFile(glbAbsPath, out unitScale, out var warnings);
                if (warnings != null) foreach (var w in warnings) Debug.LogWarning($"[MMD Hitbox][GLB] {w}");
            }
            catch (System.Exception e)
            {
                Debug.LogError($"[MMD Hitbox] GLBの読み込みに失敗しました: {e.Message}");
                return;
            }
            if (unitScale <= 0f) unitScale = 0.08f;

            // ボーン名 -> Transform。エンジンと同じく名前で解決する。
            var byName = new Dictionary<string, Transform>();
            foreach (var t in modelRoot.GetComponentsInChildren<Transform>(true))
                if (!byName.ContainsKey(t.name)) byName[t.name] = t;

            Remove(modelRoot, silent: true);   // 作り直しなので先に掃除（冪等）

            int layer = EnsureLayer(layerName);
            var container = new GameObject(RootName);
            Undo.RegisterCreatedObjectUndo(container, "Build MMD Hitboxes");
            container.transform.SetParent(modelRoot.transform, false);

            int made = 0, skippedScope = 0, unresolved = 0, unsupported = 0;
            var perBone = new Dictionary<string, int>();

            for (int i = 0; i < model.RigidBodies.Count; i++)
            {
                var rb = model.RigidBodies[i];
                if (!InScope(rb.PhysicsMode, scope)) { skippedScope++; continue; }

                if (rb.BoneIndex < 0 || rb.BoneIndex >= model.BoneNames.Count) { unresolved++; continue; }
                string boneName = model.BoneNames[rb.BoneIndex];
                if (!byName.TryGetValue(boneName, out var bone) || bone == null)
                {
                    Debug.LogWarning($"[MMD Hitbox] 剛体 '{rb.Name}' のボーン '{boneName}' が" +
                                     "モデル配下に見つかりません。この剛体は飛ばします。");
                    unresolved++; continue;
                }

                // バインド時ボーンは回転恒等・位置のみ → offset = bone^-1 * body
                var bodyWorld = RigidTransform.FromEuler(rb.Position, rb.Rotation);
                var boneWorld = new RigidTransform(Quat.Identity, model.BonePositions[rb.BoneIndex]);
                var offset = boneWorld.InverseTimes(bodyWorld);

                var go = new GameObject($"hit_{i}_{rb.Name}");
                go.transform.SetParent(bone, false);
                // 親付け後にワールドで置くと、ボーンのローカルスケールに依らず正しい姿勢になる。
                go.transform.position = bone.position +
                    bone.rotation * new Vector3(offset.Origin.x, offset.Origin.y, offset.Origin.z) * unitScale;
                go.transform.rotation = bone.rotation *
                    new Quaternion(offset.Rotation.x, offset.Rotation.y, offset.Rotation.z, offset.Rotation.w);
                if (layer != -1) go.layer = layer;

                if (!AttachShape(go, rb, unitScale, asTrigger)) { Object.DestroyImmediate(go); unsupported++; continue; }

                var mark = go.AddComponent<MmdHitbox>();
                mark.PartName = rb.Name;
                mark.BoneName = boneName;
                mark.PhysicsMode = rb.PhysicsMode;
                mark.Group = rb.Group;
                mark.RigidBodyIndex = i;

                // ヒットボックスはボーンの子でなければ追従しないので、コンテナは目印だけにして
                // 実体はボーン配下へ置く（container は除去用のハンドルとして空のまま残す）。
                perBone[boneName] = perBone.TryGetValue(boneName, out int c) ? c + 1 : 1;
                made++;
            }

            EditorUtility.SetDirty(modelRoot);

            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"[MMD Hitbox] {made} 個のヒットボックスを作りました" +
                          $"（{(asTrigger ? "トリガー" : "実体コライダー")} / Rigidbodyなし / Layer={layerName}）。");
            sb.AppendLine($"  範囲={ScopeLabelsJa[(int)scope]}  対象外={skippedScope}  ボーン未解決={unresolved}  形状未対応={unsupported}");
            sb.Append("  部位: " + string.Join(", ", perBone.OrderByDescending(kv => kv.Value).Select(kv => $"{kv.Key}×{kv.Value}")));
            Debug.Log(sb.ToString());

            if (made > 0 && asTrigger)
                Debug.Log("[MMD Hitbox] 判定の受け取り方: 相手側(弾・剣など)に Rigidbody + Collider を付け、" +
                          "OnTriggerEnter(Collider other) で other.GetComponent<MmdHitbox>().PartName を見てください。");
        }

        // PMXの形状をUnityコライダーへ。半寸法→全寸法などの換算は旧インポーターと同一。
        private static bool AttachShape(GameObject go, PmxRigidBody rb, float scale, bool asTrigger)
        {
            Collider col;
            switch (rb.ShapeType)
            {
                case 0: // 球
                    var sp = go.AddComponent<SphereCollider>();
                    sp.radius = rb.Size.x * scale;
                    col = sp;
                    break;
                case 1: // 箱（Size は半寸法なので2倍する）
                    var bx = go.AddComponent<BoxCollider>();
                    bx.size = new Vector3(rb.Size.x * 2f, rb.Size.y * 2f, rb.Size.z * 2f) * scale;
                    col = bx;
                    break;
                case 2: // カプセル（Size.x=半径, Size.y=円柱部の長さ。Unityのheightは両端の半球を含む）
                    var cp = go.AddComponent<CapsuleCollider>();
                    cp.radius = rb.Size.x * scale;
                    cp.height = (rb.Size.y + rb.Size.x * 2f) * scale;
                    cp.direction = 1; // Y軸
                    col = cp;
                    break;
                default:
                    Debug.LogWarning($"[MMD Hitbox] 剛体 '{rb.Name}' の形状 {rb.ShapeType} は未対応です。");
                    return false;
            }
            col.isTrigger = asTrigger;

            // 実体コライダーとして使う場合だけ、PMXの摩擦・反発を移す（トリガーでは無意味）。
            if (!asTrigger)
            {
                var pm = new PhysicsMaterial($"mmd_hit_{rb.Name}")
                {
                    dynamicFriction = Mathf.Clamp01(rb.Friction),
                    staticFriction = Mathf.Clamp01(rb.Friction),
                    bounciness = Mathf.Clamp01(rb.Restitution),
                    frictionCombine = PhysicsMaterialCombine.Minimum,
                    bounceCombine = PhysicsMaterialCombine.Minimum,
                };
                col.material = pm;
            }
            return true;
        }

        // ─────────────────────────────────────────────
        //  除去
        // ─────────────────────────────────────────────
        public static void Remove(GameObject modelRoot, bool silent = false)
        {
            if (modelRoot == null) return;
            int n = 0;
            foreach (var h in modelRoot.GetComponentsInChildren<MmdHitbox>(true))
            {
                if (h == null) continue;
                Undo.DestroyObjectImmediate(h.gameObject);
                n++;
            }
            foreach (var t in modelRoot.GetComponentsInChildren<Transform>(true))
                if (t != null && t.name == RootName) Undo.DestroyObjectImmediate(t.gameObject);

            if (!silent) Debug.Log($"[MMD Hitbox] {n} 個のヒットボックスを取り除きました。");
        }

        public static int Count(GameObject modelRoot) =>
            modelRoot == null ? 0 : modelRoot.GetComponentsInChildren<MmdHitbox>(true).Length;

        // ─────────────────────────────────────────────
        //  レイヤー確保（TagManager を直接編集する。空きが無ければ -1）
        // ─────────────────────────────────────────────
        private static int EnsureLayer(string layerName)
        {
            if (string.IsNullOrEmpty(layerName)) return -1;
            int existing = LayerMask.NameToLayer(layerName);
            if (existing != -1) return existing;

            var asset = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset");
            if (asset == null || asset.Length == 0) return -1;
            var tagManager = new SerializedObject(asset[0]);
            var layers = tagManager.FindProperty("layers");
            if (layers == null) return -1;

            for (int i = 8; i < layers.arraySize; i++) // 0〜7 はUnity予約
            {
                var e = layers.GetArrayElementAtIndex(i);
                if (string.IsNullOrEmpty(e.stringValue))
                {
                    e.stringValue = layerName;
                    tagManager.ApplyModifiedProperties();
                    Debug.Log($"[MMD Hitbox] レイヤー '{layerName}' を {i} 番に作りました。");
                    return i;
                }
            }
            Debug.LogWarning($"[MMD Hitbox] レイヤーの空きが無いため '{layerName}' を作れませんでした。Defaultのままにします。");
            return -1;
        }
    }
}
