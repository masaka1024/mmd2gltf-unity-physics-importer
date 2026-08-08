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

        [SerializeField] private GameObject targetPrefab;
        [SerializeField] private bool useEnglish = false; // UIラベルの言語切り替え（Consoleログは対象外）

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

        // physicsGltf（変換済み・スケール適用済み・ボーンローカル）を読めたか。
        // false のときは旧来の raw 版（PMX生値）にフォールバックしている。
        private bool usingPhysicsGltf = false;

        // ★GLBバイナリから画像を直接抽出するための作業キャッシュ（マテリアル変換時のみ使用）
        private byte[] glbBytesCache;
        private List<GltfBufferView> bufferViewsCache;
        private List<GltfImageData> imagesCache;
        private List<GltfTextureData2> gltfTexturesCache;
        private int binChunkStart = -1;
        private Dictionary<int, Texture2D> extractedTextureCache = new Dictionary<int, Texture2D>();
        private Dictionary<int, Texture2D> sharedToonCache = new Dictionary<int, Texture2D>();

        // ★調整パネル用の数値（すべて、これまで直接コードに埋め込んでいたマジックナンバー）。
        //   髪の震え・スカートの厚み・関節の遊び・輪郭線の太さなど、モデルごとに
        //   最適値が変わる部分をパネルからその場で調整できるようにする。
        private Vector2 scrollPos;                                        // ウィンドウ全体のスクロール位置
        [SerializeField] private bool showSecBodies = false;               // 調整パネルの小見出し
        [SerializeField] private bool showSecJoints = true;
        [SerializeField] private bool showSecCollision = false;
        [SerializeField] private bool showSecMaterial = false;
        [SerializeField] private bool showSecObserve = true;
        [SerializeField] private bool showTuningPanel = true;
        [SerializeField] private bool showDebugPanel = false;
        [SerializeField] private float tune_massMin = 0.01f;              // 剛体の最小質量
        [SerializeField] private float tune_inertiaScale = 1f;            // 慣性テンソルの倍率（質量は変えずに「重さ」だけ増やす）
        [SerializeField] private float tune_linearDampingMin = 0.05f;     // 剛体の最小Linear Damping
        [SerializeField] private float tune_angularDampingMin = 0.2f;     // 剛体の最小Angular Damping（髪の震え対策）
        [SerializeField] private float tune_angularDampingScale = 1f;     // Angular Damping の倍率（遅れを縮めたいときに下げる）
        [SerializeField] private float tune_linearDampingScale = 1f;      // Linear Damping の倍率

        // ★減衰の忠実変換（2026-08-06、Python側エクスポーターの単位論検証で確定）。
        //   Bullet(本家)の減衰dは「毎秒 (1-d) 倍に縮む」秒単位の定義（d=0.9=毎秒90%減）。
        //   Unityの damping は連続減衰率（毎秒およそ e^(-D) 倍）のため、0.9 を直代入すると
        //   毎秒約60%減にしかならず、本家よりかなり弱い＝慣性の質感が違う原因だった。
        //   等価変換は D = -ln(1-d)（0.9→約2.30）。ただし2026-08-06のA/B検証で、PhysXでは
        //   忠実変換ONだと全部位が硬くなり翻りが半減する(44-46°→22-28°)と実測——
        //   PhysXの律儀な拘束解きでは、本家の強減衰を上回るエネルギー注入(Bulletの緩い
        //   ソルバ由来)が無いため。よってPhysXバックエンドでは既定OFF(旧挙動=直代入)とし、
        //   本家の減衰定義はv3のBullet系ソルバで真価を発揮する忠実さトグルとして残す。
        [SerializeField] private bool  tune_damperBulletFaithful = false; // Bullet秒単位→Unity連続減衰率の換算（PhysXでは既定OFF）
        [SerializeField] private float tune_springRotFloor = 3f;          // 関節ばねの最低値
        [SerializeField] private float tune_springDamperRatio = 0.1f;     // ダンパー＝ばね×この比率（ばね0ならダンパーも0）
        [SerializeField] private bool  tune_normalizeDriveByInertia = true; // ドライブを剛体の慣性で正規化する
        [SerializeField] private float tune_driveFreqHz = 2f;             // 戻ろうとする速さ（Hz）。全剛体で共通になる
        [SerializeField] private float tune_driveDampingRatio = 0.5f;     // 減衰比ζ（1で臨界減衰）
        [SerializeField] private float tune_skirtColliderScale = 1f;    // スカートのコライダー縮小率
        [SerializeField] private float tune_hairColliderScale = 1f;       // 髪のコライダー縮小率
        [SerializeField] private float tune_maxDepenetrationVelocity = 1f; // めり込み解消速度の上限（衝突再有効化時の弾け防止）
        [SerializeField] private float tune_angularSlackDeg = 0f;        // 関節角度制限の遊び（度）
        [SerializeField] private float tune_linearSlackScale = 1f;       // 移動制限の倍率（横リング・房どうしの遊び）
        [SerializeField] private float tune_skirtLinearScale = 8f;       // ★スカート横リング専用の直線倍率（円周の伸び代。スイープH条件の実証値=8。髪の房には掛からない）
        [SerializeField] private float tune_limitSoftness = 0f;          // 制限を柔らかくする（0=硬い壁、Bulletは柔らかい）

        // ★スカートのソフトリミット（2026-08-01の「翻り」調査で確立）。
        //   「25°の見えない天井」の主犯は縦（段間）ジョイントの硬いリミットだった。
        //   取付＋縦のY/Zリミット角を広げ、壁を弱いばねに置き換えると翻り（総曲げ40°超）が出る。
        //   既存の tune_limitSoftness（全ジョイント一律・最弱でもばね200）とは別系統で、
        //   こちらは実測で当たりだった「ばね5」級の思い切った柔らかさをスカート限定で使う。
        //   横リング（直線可動あり）は対象外：完全に緩めると静止時に継ぎ目が開くため。
        [SerializeField] private bool   tune_softLimitSkirt = true;       // スカートのソフトリミットを使う
        [SerializeField] private float  tune_softLimitSpring = 2f;        // リミットばねの強さ（実測の当たり値=5）
        [SerializeField] private float  tune_softLimitDamper = 0.1f;     // リミットばねの減衰。慣性正規化を通らない生の値のため軽量パネルには小さく（翻り調査で1はブレーキ過大と判明。バタつく場合のみ微増）
        [SerializeField] private float  tune_softLimitScale = 3f;         // Y/Zリミット角の拡大倍率（実測で2.0以上は飽和）
        [SerializeField] private bool   tune_softLimitVertical = true;    // 縦（段間）ジョイントにも適用（主犯なので既定ON）
        [SerializeField] private bool   tune_skirtYawTight = true;        // ★ヨー軸(鉛直に最も近い角度軸)は広げない：本家のヨー遅れ1-3°(共回転)を再現し遠心力を確保する
        [SerializeField] private string tune_collisionForceActiveFilter = "スカート"; // この名を含む組は埋まり保留にしない（空=無効）
        [SerializeField] private bool  tune_useCollisionMask = true;      // PMXのグループ／マスクをペア単位で再現する
        [SerializeField] private bool  tune_flipHandedness = true;        // glTF(右手系)→Unity(左手系)のZ反転を掛ける
        [SerializeField] private bool  tune_matchMmdGravity = true;       // 重力をMMDの単位系に合わせる（unitScale倍）
        [SerializeField] private float tune_gravityScale = 0.8f;          // 重力の倍率（自動検出は unitScale × 10）
        [SerializeField] private float tune_gravityBaseScale = 10f;       // MMD側の GravityBaseScale（本家の既定は10）
        [SerializeField] private float tune_warmupSeconds = 0.2f;         // 再生直後に物理を止めておく秒数
        [SerializeField] private float tune_contactOffsetRatio = 0.1f;    // 接触オフセット＝剛体の最小寸法×この比率
        [SerializeField] private int   tune_collisionDetection = 1;       // 0=Discrete 1=ContinuousSpeculative 2=ContinuousDynamic
        [SerializeField] private bool  tune_useJointProjection = false;    // 解き残りをジョイントの投影で引き戻す
        [SerializeField] private float tune_projectionAngleDeg = 10f;     // 投影を発動させる角度のずれ（度）

        // ★部位別ダイヤル（2026-08-01の実ダンス数値診断より）。
        //   全身共通のHz/ζ/遊びでは「スカートは硬くて開き不足、前髪は柔らかすぎて過剰」が
        //   同時には解決できないと確定したため、名前パターン（スカート／前髪／もみあげ）ごとに
        //   tune_driveFreqHz・tune_driveDampingRatio・tune_angularSlackDeg への倍率／オフセットを
        //   個別に持たせる。初期値は診断時点での初期案。
        [SerializeField] private bool  tune_usePartDials = true;          // 部位別ダイヤルを使う

        // ★Swing/Twist分離(2026-08-01)：PhysXのConfigurableJointはY/Z(Swing)が
        //   円錐で結合される仕様のため、Bullet(本家)の軸ごと独立した角度制限より
        //   実効可動域が狭くなる。VMDビューアのリバースエンジニアリング結果で、
        //   本家は一般の剛体+ジョイント(Bulletの6DOF、軸ごと独立)で動いており、
        //   スカート専用の特殊処理は無いと判明→PhysX側の円錐結合が「翻らない」の
        //   主因という仮説のもと、スカート⇔下半身の「取付」ジョイントだけ、
        //   Y単軸→X単軸→Z単軸の3連ヒンジ(軽量な中継剛体2つを挟む)に組み替える。
        [SerializeField] private bool  tune_decoupleSkirtSwingTwist = false; // 取付ジョイントをY/X/Z単軸3連に分離
        [SerializeField] private float tune_skirtHzScale = 0.38f;         // スカート：戻る速さの倍率
        [SerializeField] private float tune_skirtZetaScale = 0.39f;       // スカート：減衰比の倍率
        [SerializeField] private float tune_skirtSlackDeg = 19.6f;          // スカート：角度遊びの追加オフセット(度)
        [SerializeField] private float tune_bangsHzScale = 1.5f;         // 前髪：戻る速さの倍率
        [SerializeField] private float tune_bangsZetaScale = 0.7f;       // 前髪：減衰比の倍率
        [SerializeField] private float tune_bangsSlackDeg = -10f;        // 前髪：角度遊びの追加オフセット(度)
        [SerializeField] private float tune_sideburnsHzScale = 1.2f;     // もみあげ：戻る速さの倍率
        [SerializeField] private float tune_sideburnsZetaScale = 1f;     // もみあげ：減衰比の倍率
        [SerializeField] private float tune_sideburnsSlackDeg = -5f;     // もみあげ：角度遊びの追加オフセット(度)

        // ★スライダーを動かしたあと再構築を忘れないよう、設定の指紋を記録しておく。
        //   ボタンを押した時点の指紋と現在の指紋が違えば「要再構築」として色を変える。
        [SerializeField] private string builtRbSignature = "";
        [SerializeField] private string builtJointSignature = "";

        // ── 観測用（挙動には影響しない） ──
        [SerializeField] private float watch_moveThreshold = 0.3f;         // 発散とみなす移動量(m)
        [SerializeField] private bool  watch_pauseOnFirst = true;          // 最初の発散でエディタを一時停止
        [SerializeField] private float tune_outlineWidthFactor = 0.08f;   // edgeSize→lilToon _OutlineWidthへの換算係数

        private void OnGUI()
        {
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label(L("MMD glTF 統合インポーター (Rigidbody直付け版)", "MMD glTF Unity Importer (Rigidbody-on-Bone)"), EditorStyles.boldLabel);
            GUILayout.FlexibleSpace();
            useEnglish = GUILayout.Toggle(useEnglish, useEnglish ? "EN" : "日本語", "Button", GUILayout.Width(60));
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.Space();

            // ★ウィンドウが縦に伸びても操作できるよう全体をスクロールさせる
            scrollPos = EditorGUILayout.BeginScrollView(scrollPos);

            targetPrefab = (GameObject)EditorGUILayout.ObjectField(
                L("対象モデル (Scene上)", "Target Model (in Scene)"), targetPrefab, typeof(GameObject), true);

            EditorGUILayout.Space();

            showTuningPanel = EditorGUILayout.Foldout(showTuningPanel, L("調整パネル（髪・スカート・関節・輪郭線）", "Tuning Panel (Hair / Skirt / Joints / Outline)"), true);
            if (showTuningPanel)
            {
                EditorGUI.indentLevel++;
                showSecBodies = EditorGUILayout.Foldout(showSecBodies, L("剛体の基本設定", "Rigidbody Basics"), true);
                if (showSecBodies) {
                tune_massMin = EditorGUILayout.Slider(L("最小質量", "Min Mass"), tune_massMin, 0.001f, 1f);
                tune_inertiaScale = EditorGUILayout.Slider(L("慣性の倍率（重さの手応え）", "Inertia Scale"), tune_inertiaScale, 0.5f, 20f);
                tune_linearDampingMin = EditorGUILayout.Slider(L("最小Linear Damping", "Min Linear Damping"), tune_linearDampingMin, 0f, 2f);
                tune_angularDampingMin = EditorGUILayout.Slider(L("最小Angular Damping（震え対策）", "Min Angular Damping (anti-jitter)"), tune_angularDampingMin, 0f, 2f);
                tune_angularDampingScale = EditorGUILayout.Slider(L("Angular Damping の倍率（遅れ）", "Angular Damping Scale"), tune_angularDampingScale, 0f, 2f);
                tune_linearDampingScale = EditorGUILayout.Slider(L("Linear Damping の倍率", "Linear Damping Scale"), tune_linearDampingScale, 0f, 2f);
                tune_damperBulletFaithful = EditorGUILayout.Toggle(L("減衰をBullet忠実変換 D=-ln(1-d)", "Bullet-Faithful Damping D=-ln(1-d)"), tune_damperBulletFaithful);

                }

                showSecJoints = EditorGUILayout.Foldout(showSecJoints, L("関節・ばね（硬さと遅れ）", "Joints & Springs"), true);
                if (showSecJoints) {
                using (new EditorGUI.DisabledScope(tune_normalizeDriveByInertia))
                    tune_springRotFloor = EditorGUILayout.Slider(L("ばねの最低値", "Min Spring Strength"), tune_springRotFloor, 0f, 20f);
                tune_normalizeDriveByInertia = EditorGUILayout.Toggle(L("ドライブを慣性で正規化する", "Normalize Drive by Inertia"), tune_normalizeDriveByInertia);
                using (new EditorGUI.DisabledScope(!tune_normalizeDriveByInertia))
                {
                    tune_driveFreqHz = EditorGUILayout.Slider(L("戻ろうとする速さ(Hz)", "Restore Frequency (Hz)"), tune_driveFreqHz, 0f, 10f);
                    tune_driveDampingRatio = EditorGUILayout.Slider(L("減衰比ζ", "Damping Ratio"), tune_driveDampingRatio, 0f, 2f);
                }
                using (new EditorGUI.DisabledScope(tune_normalizeDriveByInertia))
                    tune_springDamperRatio = EditorGUILayout.Slider(L("ばねに対する減衰の比率", "Damper / Spring Ratio"), tune_springDamperRatio, 0f, 1f);
                tune_angularSlackDeg = EditorGUILayout.Slider(L("角度制限の遊び(度)", "Angular Limit Slack (deg)"), tune_angularSlackDeg, 0f, 90f);
                tune_linearSlackScale = EditorGUILayout.Slider(L("移動制限の倍率（横のつながりの遊び）", "Linear Limit Scale"), tune_linearSlackScale, 1f, 10f);
                tune_skirtLinearScale = EditorGUILayout.Slider(L("スカート横リングの直線倍率（円周の伸び代）", "Skirt Ring Linear Scale"), tune_skirtLinearScale, 1f, 12f);
                tune_limitSoftness = EditorGUILayout.Slider(L("制限の柔らかさ(0=硬い壁)", "Limit Softness (0 = hard)"), tune_limitSoftness, 0f, 1f);

                EditorGUILayout.Space();
                tune_softLimitSkirt = EditorGUILayout.Toggle(L("スカートのソフトリミット（翻り対策）", "Skirt Soft Limit (flare fix)"), tune_softLimitSkirt);
                using (new EditorGUI.DisabledScope(!tune_softLimitSkirt))
                {
                    tune_softLimitSpring = EditorGUILayout.Slider(L("　リミットばねの強さ", "  Limit Spring"), tune_softLimitSpring, 1f, 200f);
                    tune_softLimitDamper = EditorGUILayout.Slider(L("　リミットばねの減衰", "  Limit Damper"), tune_softLimitDamper, 0f, 40f);
                    tune_softLimitScale = EditorGUILayout.Slider(L("　リミット角の拡大倍率", "  Limit Angle Scale"), tune_softLimitScale, 1f, 4f);
                    tune_softLimitVertical = EditorGUILayout.Toggle(L("　縦（段間）ジョイントにも適用", "  Apply to Vertical (inter-row)"), tune_softLimitVertical);
                    tune_skirtYawTight = EditorGUILayout.Toggle(L("　ヨー軸は締める（共回転＝遠心力の確保）", "  Keep Yaw Tight (co-rotation)"), tune_skirtYawTight);
                }
                tune_collisionForceActiveFilter = EditorGUILayout.TextField(L("埋まり保留にしない名前（衝突マスク）", "Never-defer name filter (collision)"), tune_collisionForceActiveFilter);
                tune_useJointProjection = EditorGUILayout.Toggle(L("解き残りを投影で引き戻す", "Use Joint Projection"), tune_useJointProjection);
                using (new EditorGUI.DisabledScope(!tune_useJointProjection))
                    tune_projectionAngleDeg = EditorGUILayout.Slider(L("投影する角度ずれ(度)", "Projection Angle (deg)"), tune_projectionAngleDeg, 1f, 60f);

                EditorGUILayout.Space();
                tune_usePartDials = EditorGUILayout.Toggle(L("部位別ダイヤルを使う（スカート/前髪/もみあげ）", "Use Per-Part Dials (Skirt/Bangs/Sideburns)"), tune_usePartDials);
                EditorGUILayout.Space();
                tune_decoupleSkirtSwingTwist = EditorGUILayout.Toggle(L("スカート取付をY/X/Z単軸3連ヒンジに分離", "Decouple skirt attachment into Y/X/Z single-axis hinges"), tune_decoupleSkirtSwingTwist);
                using (new EditorGUI.DisabledScope(!tune_usePartDials))
                {
                    EditorGUILayout.LabelField(L("スカート（名前に「スカート」を含む）", "Skirt (name contains \"スカート\")"), EditorStyles.miniBoldLabel);
                    EditorGUI.indentLevel++;
                    tune_skirtHzScale = EditorGUILayout.Slider(L("Hz倍率", "Hz Scale"), tune_skirtHzScale, 0f, 3f);
                    tune_skirtZetaScale = EditorGUILayout.Slider(L("ζ倍率", "Zeta Scale"), tune_skirtZetaScale, 0f, 3f);
                    tune_skirtSlackDeg = EditorGUILayout.Slider(L("遊びの追加オフセット(度)", "Slack Offset (deg)"), tune_skirtSlackDeg, -30f, 30f);
                    EditorGUI.indentLevel--;

                    EditorGUILayout.LabelField(L("前髪（名前に「前髪」を含む）", "Bangs (name contains \"前髪\")"), EditorStyles.miniBoldLabel);
                    EditorGUI.indentLevel++;
                    tune_bangsHzScale = EditorGUILayout.Slider(L("Hz倍率", "Hz Scale"), tune_bangsHzScale, 0f, 3f);
                    tune_bangsZetaScale = EditorGUILayout.Slider(L("ζ倍率", "Zeta Scale"), tune_bangsZetaScale, 0f, 3f);
                    tune_bangsSlackDeg = EditorGUILayout.Slider(L("遊びの追加オフセット(度)", "Slack Offset (deg)"), tune_bangsSlackDeg, -30f, 30f);
                    EditorGUI.indentLevel--;

                    EditorGUILayout.LabelField(L("もみあげ（名前に「もみあげ」「モミアゲ」を含む）", "Sideburns (name contains \"もみあげ\"/\"モミアゲ\")"), EditorStyles.miniBoldLabel);
                    EditorGUI.indentLevel++;
                    tune_sideburnsHzScale = EditorGUILayout.Slider(L("Hz倍率", "Hz Scale"), tune_sideburnsHzScale, 0f, 3f);
                    tune_sideburnsZetaScale = EditorGUILayout.Slider(L("ζ倍率", "Zeta Scale"), tune_sideburnsZetaScale, 0f, 3f);
                    tune_sideburnsSlackDeg = EditorGUILayout.Slider(L("遊びの追加オフセット(度)", "Slack Offset (deg)"), tune_sideburnsSlackDeg, -30f, 30f);
                    EditorGUI.indentLevel--;
                }

                }

                showSecCollision = EditorGUILayout.Foldout(showSecCollision, L("コライダー・衝突", "Colliders & Collision"), true);
                if (showSecCollision) {
                tune_flipHandedness = EditorGUILayout.Toggle(L("glTFの座標系を変換する(Z反転)", "Convert glTF Handedness (flip Z)"), tune_flipHandedness);
                tune_matchMmdGravity = EditorGUILayout.Toggle(L("重力をMMDの単位系に合わせる", "Match MMD Gravity Scale"), tune_matchMmdGravity);
                using (new EditorGUI.DisabledScope(!tune_matchMmdGravity))
                    tune_gravityScale = EditorGUILayout.Slider(L("重力の倍率", "Gravity Scale"), tune_gravityScale, 0.05f, 3f);
                    tune_gravityBaseScale = EditorGUILayout.Slider(L("MMDのGravityBaseScale", "MMD GravityBaseScale"), tune_gravityBaseScale, 1f, 20f);
                tune_warmupSeconds = EditorGUILayout.Slider(L("物理の始動を遅らせる(秒)", "Physics Warmup (sec)"), tune_warmupSeconds, 0f, 1f);
                tune_useCollisionMask = EditorGUILayout.Toggle(L("PMXの衝突マスクを使う", "Use PMX Collision Mask"), tune_useCollisionMask);
                tune_contactOffsetRatio = EditorGUILayout.Slider(L("接触オフセット比（剛体サイズ比）", "Contact Offset Ratio"), tune_contactOffsetRatio, 0.02f, 1f);
                tune_collisionDetection = EditorGUILayout.Popup(L("衝突検出方式", "Collision Detection"), tune_collisionDetection,
                    new string[] { "Discrete", "Continuous Speculative", "Continuous Dynamic" });
                tune_skirtColliderScale = EditorGUILayout.Slider(L("スカートのコライダー縮小率", "Skirt Collider Scale"), tune_skirtColliderScale, 0.1f, 1.5f);
                tune_hairColliderScale = EditorGUILayout.Slider(L("髪のコライダー縮小率", "Hair Collider Scale"), tune_hairColliderScale, 0.1f, 1.5f);
                tune_maxDepenetrationVelocity = EditorGUILayout.Slider(L("めり込み解消速度の上限", "Max Depenetration Velocity"), tune_maxDepenetrationVelocity, 0.02f, 10f);

                }

                showSecMaterial = EditorGUILayout.Foldout(showSecMaterial, L("マテリアル", "Materials"), true);
                if (showSecMaterial) {
                tune_outlineWidthFactor = EditorGUILayout.Slider(L("輪郭線の太さ換算係数", "Outline Width Factor"), tune_outlineWidthFactor, 0.01f, 0.3f);

                }

                if (GUILayout.Button(L("既定値に戻す", "Reset to Defaults")))
                {
                    tune_massMin = 0.01f;
                    tune_inertiaScale = 1f;
                    tune_linearDampingMin = 0.05f;
                    tune_angularDampingMin = 0.2f;
                    tune_angularDampingScale = 1f;
                    tune_linearDampingScale = 1f;
                    tune_damperBulletFaithful = false;
                    tune_springRotFloor = 3f;
                    tune_springDamperRatio = 0.1f;
                    tune_normalizeDriveByInertia = true;
                    tune_driveFreqHz = 2f;
                    tune_driveDampingRatio = 0.5f;
                    tune_useCollisionMask = true;
                    tune_warmupSeconds = 0.2f;
                    tune_flipHandedness = true;
                    tune_matchMmdGravity = true;
                    tune_gravityScale = 0.8f;
                    tune_gravityBaseScale = 10f;
                    tune_contactOffsetRatio = 0.1f;
                    tune_collisionDetection = 1;
                    tune_skirtColliderScale = 1f;
                    tune_hairColliderScale = 1f;
                    tune_maxDepenetrationVelocity = 1f;
                    tune_angularSlackDeg = 0f;
                    tune_linearSlackScale = 1f;
                    tune_skirtLinearScale = 8f;
                    tune_limitSoftness = 0f;
                    tune_softLimitSkirt = true;
                    tune_softLimitSpring = 2f;
                    tune_softLimitDamper = 0.1f;
                    tune_softLimitScale = 3f;
                    tune_softLimitVertical = true;
                    tune_skirtYawTight = true;
                    tune_collisionForceActiveFilter = "スカート";
                    tune_useJointProjection = false;
                    tune_projectionAngleDeg = 10f;
                    tune_outlineWidthFactor = 0.08f;
                    tune_usePartDials = true;
                    tune_decoupleSkirtSwingTwist = false;
                    tune_skirtHzScale = 0.38f;
                    tune_skirtZetaScale = 0.39f;
                    tune_skirtSlackDeg = 19.6f;
                    tune_bangsHzScale = 1.5f;
                    tune_bangsZetaScale = 0.7f;
                    tune_bangsSlackDeg = -10f;
                    tune_sideburnsHzScale = 1.2f;
                    tune_sideburnsZetaScale = 1f;
                    tune_sideburnsSlackDeg = -5f;
                }
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.Space();

            GUILayout.Label(L("【1】物理エンジンの構築", "[1] Build Physics Engine"), EditorStyles.miniBoldLabel);

            bool rbDirty = builtRbSignature != RbSignature();
            bool jointDirty = rbDirty || builtJointSignature != JointSignature();

            if (DirtyButton(rbDirty, "1. 剛体とコライダーを配置 (古いエラーも自動掃除)", "1. Generate Rigidbodies & Colliders (auto-cleans old errors)")
                && targetPrefab != null)
            {
                GenerateRigidBodies();
                builtRbSignature = RbSignature();
                builtJointSignature = ""; // 剛体を作り直したらジョイントも要再構築
            }

            if (DirtyButton(jointDirty, "2. ジョイントを結合 (必ず1の直後に実行)", "2. Connect Joints (run right after step 1)")
                && targetPrefab != null)
            {
                ConnectJoints();
                builtJointSignature = JointSignature();
            }

            if (rbDirty || jointDirty)
                EditorGUILayout.HelpBox(
                    L("設定が変わっています。橙色のボタンを上から順に押し直してください。",
                      "Settings changed. Press the highlighted buttons in order."),
                    MessageType.Info);

            EditorGUILayout.Space();

            GUILayout.Label(L("【2】描画・見た目の修正", "[2] Rendering & Appearance"), EditorStyles.miniBoldLabel);
            if (GUILayout.Button(L("3. マテリアルをlilToonへ変換（トゥーン/スフィア復元）", "3. Convert Materials to lilToon (restore Toon/Sphere maps)")) && targetPrefab != null)
            {
                ConvertMaterialsToLilToon();
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

                if (GUILayout.Button(L("物理シーンを診断", "Diagnose Physics Scene")) && targetPrefab != null)
                {
                    DiagnosePhysics();
                }

                EditorGUILayout.Space();
                showSecObserve = EditorGUILayout.Foldout(showSecObserve, L("― 観測 ―", "- Observation -"), true);
                if (showSecObserve) {

                if (GUILayout.Button(L("セットアップを検証（再生前・ログ出力）", "Validate Setup (before Play, logs)")) && targetPrefab != null)
                {
                    ValidatePhysicsSetup();
                }

                watch_moveThreshold = EditorGUILayout.Slider(L("発散とみなす移動量(m)", "Divergence Move Threshold (m)"), watch_moveThreshold, 0.05f, 2f);
                watch_pauseOnFirst = EditorGUILayout.Toggle(L("最初の発散で一時停止", "Pause on First Divergence"), watch_pauseOnFirst);

                if (GUILayout.Button(L("アニメと物理の競合を確認", "Check Animation vs Physics Conflict")) && targetPrefab != null)
                {
                    ValidateAnimationConflict();
                }

                if (GUILayout.Button(L("アニメーションを物理と同期させる", "Sync Animation to Physics")) && targetPrefab != null)
                {
                    SyncAnimationToPhysics();
                }

                if (GUILayout.Button(L("コライダーの向きを検証", "Validate Collider Orientation")) && targetPrefab != null)
                {
                    ValidateColliderOrientation();
                }

                if (GUILayout.Button(L("静止姿勢の食い込みを計測", "Measure Rest-Pose Overlap")) && targetPrefab != null)
                {
                    ValidateRestOverlap();
                }

                if (GUILayout.Button(L("揺れの計測を付与 / 除去（本家と数値比較）", "Add / Remove Motion Stats (compare with MMD)")) && targetPrefab != null)
                {
                    ToggleMotionStats();
                }

                if (GUILayout.Button(L("実行時ウォッチャーを付与 / 除去", "Add / Remove Runtime Watcher")) && targetPrefab != null)
                {
                    ToggleRuntimeWatcher();
                }

                }

                if (GUILayout.Button(L("指定キーワードのボーン回転を一括出力（再生中/一時停止中も可）", "Dump Bone Rotations by Keyword (works during Play/Pause)")) && targetPrefab != null)
                {
                    DumpBoneRotationsByKeyword("スカート");
                }

                if (GUILayout.Button(L("ジョイントの接続関係を一覧出力（髪）", "Dump Joint Connections (Hair)")) && targetPrefab != null)
                {
                    DumpJointConnections("髪");
                }

                if (GUILayout.Button(L("剛体JSONのmode値を一括確認（スカート）", "Dump Rigidbody 'mode' Values (Skirt)")) && targetPrefab != null)
                {
                    DumpRigidBodyModes("スカート");
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

        // 剛体・コライダーの生成結果に影響する設定の指紋
        private string RbSignature()
        {
            return string.Join("|", new string[]{
                (targetPrefab != null ? targetPrefab.GetHashCode().ToString() : "-"),
                tune_massMin.ToString("F4"), tune_inertiaScale.ToString("F3"),
                tune_linearDampingMin.ToString("F3"), tune_angularDampingMin.ToString("F3"),
                tune_linearDampingScale.ToString("F3"), tune_angularDampingScale.ToString("F3"),
                tune_damperBulletFaithful ? "1" : "0",
                tune_skirtColliderScale.ToString("F3"), tune_hairColliderScale.ToString("F3"),
                tune_maxDepenetrationVelocity.ToString("F3"), tune_contactOffsetRatio.ToString("F3"),
                tune_collisionDetection.ToString(), tune_useCollisionMask ? "1" : "0",
                tune_flipHandedness ? "1" : "0", tune_matchMmdGravity ? "1" : "0",
                tune_gravityScale.ToString("F3"), tune_gravityBaseScale.ToString("F2"),
                tune_warmupSeconds.ToString("F3"),
            });
        }

        // ジョイントの結合結果に影響する設定の指紋（剛体側が変われば当然こちらも作り直し）
        private string JointSignature()
        {
            return RbSignature() + "#" + string.Join("|", new string[]{
                tune_springRotFloor.ToString("F3"), tune_springDamperRatio.ToString("F3"),
                tune_normalizeDriveByInertia ? "1" : "0",
                tune_driveFreqHz.ToString("F3"), tune_driveDampingRatio.ToString("F3"),
                tune_angularSlackDeg.ToString("F2"), tune_linearSlackScale.ToString("F2"), tune_skirtLinearScale.ToString("F2"),
                tune_limitSoftness.ToString("F3"),
                tune_softLimitSkirt ? "1" : "0",
                tune_softLimitSpring.ToString("F2"), tune_softLimitDamper.ToString("F2"),
                tune_softLimitScale.ToString("F2"), tune_softLimitVertical ? "1" : "0",
                tune_skirtYawTight ? "1" : "0",
                tune_collisionForceActiveFilter ?? "",
                tune_useJointProjection ? "1" : "0", tune_projectionAngleDeg.ToString("F2"),
                tune_usePartDials ? "1" : "0",
                tune_decoupleSkirtSwingTwist ? "1" : "0",
                tune_skirtHzScale.ToString("F3"), tune_skirtZetaScale.ToString("F3"), tune_skirtSlackDeg.ToString("F2"),
                tune_bangsHzScale.ToString("F3"), tune_bangsZetaScale.ToString("F3"), tune_bangsSlackDeg.ToString("F2"),
                tune_sideburnsHzScale.ToString("F3"), tune_sideburnsZetaScale.ToString("F3"), tune_sideburnsSlackDeg.ToString("F2"),
            });
        }

        // ★Bullet(本家)の秒単位減衰 d を Unity の連続減衰率 D へ等価換算する。
        //   Bullet: v ∝ (1-d)^t（d=0.9 なら毎秒90%減） / Unity: v ∝ おおよそ e^(-D·t)
        //   等価条件 e^(-D) = 1-d より D = -ln(1-d)。d=0.9 → D≈2.30。
        //   d=1.0 は ln が発散するため 0.999 でクランプ（D≈6.9）。
        private static float BulletToUnityDamping(float d)
        {
            d = Mathf.Clamp(d, 0f, 0.999f);
            return -Mathf.Log(1f - d);
        }

        // 要再構築なら色を変え、ラベルに印を付けてボタンを描く
        private bool DirtyButton(bool dirty, string ja, string en)
        {
            Color prev = GUI.backgroundColor;
            if (dirty) GUI.backgroundColor = new Color(1.0f, 0.72f, 0.30f); // 琥珀色＝要再構築
            bool clicked = GUILayout.Button(L(ja, en) + (dirty ? L("　★要再構築", "  * needs rebuild") : ""));
            GUI.backgroundColor = prev;
            return clicked;
        }

        // ★UIラベル用の簡易ローカライズヘルパー。useEnglishがtrueなら英語、falseなら日本語を返す。
        //   Consoleログ(Debug.Log等)はこの対象外で、日本語のまま。
        private string L(string ja, string en) => useEnglish ? en : ja;

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

        // ★extras.mmd には「raw版(rigidBodies/joints, PMX生値)」と「変換済み
        //   physicsGltf 版」の2系統の剛体・ジョイントが併存する(extrasMmd_schema.md
        //   参照)。本インポーターは raw 版を使う設計のため、"physicsGltf" キーより
        //   前の範囲だけを検索対象にし、エクスポーター側のキー出力順が変わっても
        //   誤って physicsGltf 側の配列を拾わないようにする。
        private string GetRawPhysicsSearchText()
        {
            string jsonText = GetRawJsonText();
            if (string.IsNullOrEmpty(jsonText)) return jsonText;
            int cut = jsonText.IndexOf("\"physicsGltf\"", StringComparison.Ordinal);
            return cut >= 0 ? jsonText.Substring(0, cut) : jsonText;
        }

        // ★physicsGltf セクション（変換済み・スケール適用済み・ボーンローカル）だけを返す。
        //   physicsGltf_schema.md 参照。距離は glTF シーン単位（unitScale 適用済み）、
        //   回転はクォータニオン、位置は bone のローカル空間で入っているため、
        //   インポーター側でのスケール推定・座標変換・オイラー順序の解釈が不要になる。
        private string GetPhysicsGltfText()
        {
            string jsonText = GetRawJsonText();
            if (string.IsNullOrEmpty(jsonText)) return null;
            int cut = jsonText.IndexOf("\"physicsGltf\"", StringComparison.Ordinal);
            return cut >= 0 ? jsonText.Substring(cut) : null;
        }

        // physicsGltf の unitScale（適用済みスケール）を読む。見つからなければ 0。
        private float ReadUnitScale()
        {
            string pg = GetPhysicsGltfText();
            if (string.IsNullOrEmpty(pg)) return 0f;
            int i = pg.IndexOf("\"unitScale\"", StringComparison.Ordinal);
            if (i < 0) return 0f;
            int colon = pg.IndexOf(':', i);
            if (colon < 0) return 0f;
            int j = colon + 1;
            while (j < pg.Length && (pg[j] == ' ' || pg[j] == '\t')) j++;
            int start = j;
            while (j < pg.Length && (char.IsDigit(pg[j]) || pg[j] == '.' || pg[j] == '-' || pg[j] == 'e' || pg[j] == 'E' || pg[j] == '+')) j++;
            float v;
            return float.TryParse(pg.Substring(start, j - start), System.Globalization.NumberStyles.Float,
                                  System.Globalization.CultureInfo.InvariantCulture, out v) ? v : 0f;
        }

        private List<RigidBodyData> ExtractRigidBodies()
        {
            usingPhysicsGltf = false;

            // ── ① physicsGltf 版を優先 ──
            string pgText = GetPhysicsGltfText();
            if (!string.IsNullOrEmpty(pgText))
            {
                string pgArr = FindJsonArray(pgText, "rigidBodies");
                if (!string.IsNullOrEmpty(pgArr))
                {
                    var pgWrap = JsonUtility.FromJson<RigidBodyListWrapper>("{\"list\":" + pgArr + "}");
                    if (pgWrap != null && pgWrap.list != null && pgWrap.list.Count > 0)
                    {
                        foreach (var d in pgWrap.list) d.AdoptPhysicsGltfFields();
                        usingPhysicsGltf = true;
                        Debug.Log($"[MMD Physics] physicsGltf から剛体 {pgWrap.list.Count} 件を読みました（スケール適用済み・ボーンローカル）。");
                        return pgWrap.list;
                    }
                }
            }

            // ── ② 旧形式（raw / PMX生値）へフォールバック ──
            var bodiesList = new List<RigidBodyData>();
            string jsonText = GetRawPhysicsSearchText();
            if (string.IsNullOrEmpty(jsonText)) return bodiesList;

            string rbArrayText = FindJsonArray(jsonText, "rigidBodies");
            if (string.IsNullOrEmpty(rbArrayText)) rbArrayText = FindJsonArray(jsonText, "rigid_bodies");

            if (!string.IsNullOrEmpty(rbArrayText))
            {
                var wrapper = JsonUtility.FromJson<RigidBodyListWrapper>("{\"list\":" + rbArrayText + "}");
                if (wrapper != null && wrapper.list != null) bodiesList = wrapper.list;
            }
            if (bodiesList.Count > 0)
                Debug.LogWarning($"[MMD Physics] physicsGltf が見つからないため raw(PMX生値) から剛体 {bodiesList.Count} 件を読みました。スケールは推定値になります。新しいエクスポーターで再出力すると精度が上がります。");
            return bodiesList;
        }

        // ★joints 抽出：オブジェクト型の配列だけを狙う（skinの整数配列を回避）
        private List<JointData> ExtractJoints()
        {
            var list = new List<JointData>();

            // ── ① physicsGltf 版を優先（角度制限はglTF軸へ鏡映済み、移動制限はglTF単位）──
            if (usingPhysicsGltf)
            {
                string pgText = GetPhysicsGltfText();
                if (!string.IsNullOrEmpty(pgText))
                {
                    string pgArr = FindJsonArrayObjects(pgText, "joints");
                    if (!string.IsNullOrEmpty(pgArr))
                    {
                        var pgWrap = JsonUtility.FromJson<JointListWrapper>("{\"list\":" + pgArr + "}");
                        if (pgWrap != null && pgWrap.list != null && pgWrap.list.Count > 0)
                        {
                            foreach (var d in pgWrap.list) d.AdoptPhysicsGltfFields();
                            Debug.Log($"[MMD Physics] physicsGltf からジョイント {pgWrap.list.Count} 件を読みました。");
                            return pgWrap.list;
                        }
                    }
                }
                Debug.LogWarning("[MMD Physics] 剛体は physicsGltf から読めましたが joints が見つかりません。単位系が混ざるのを避けるため接続をスキップします。");
                return list;
            }

            // ── ② 旧形式（raw / PMX生値）──
            string jsonText = GetRawPhysicsSearchText();
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

            // 物理剛体専用レイヤーを用意
            int mmdLayer = EnsureLayer("MMDPhysics");

            // ★マスク方式では、どの組が当たるかはPMXのグループ／マスクが決める。
            //   レイヤー側は全部同じレイヤーにまとめて自己衝突を「有効」にしておき、
            //   個々の組を MmdCollisionMask が Physics.IgnoreCollision で落とす。
            if (tune_useCollisionMask)
            {
                if (mmdLayer != -1) SetLayerCollision(mmdLayer, mmdLayer, false);
            }
            else if (mmdLayer != -1)
            {
                SetLayerCollision(mmdLayer, mmdLayer, true); // 自己衝突を無効化（同士の絡まり暴れを防ぐ）
            }

            // ★スカートだけ専用レイヤーに分離する。
            //   スカート同士は従来どおり衝突なし（同じ鎖内での絡まり暴れを防ぐ）が、
            //   スカート⇔体・脚（MMDPhysicsレイヤー）の衝突は復活させる。
            int skirtLayer = -1;
            if (!tune_useCollisionMask)
            {
                skirtLayer = EnsureLayer("MMDPhysicsSkirt");
                if (skirtLayer != -1) SetLayerCollision(skirtLayer, skirtLayer, true);  // スカート同士は無効のまま
                if (skirtLayer != -1 && mmdLayer != -1) SetLayerCollision(skirtLayer, mmdLayer, false); // スカート⇔体は有効化
            }

            // ★髪も同じ考え方で専用レイヤーに分離する。
            //   髪同士は無効のまま（絡まり暴れ防止）だが、髪⇔体・頭（MMDPhysics）の
            //   衝突は復活させる。これで髪が顔を突き抜けず、頭の丸みで受け止められる。
            int hairLayer = -1;
            if (!tune_useCollisionMask)
            {
                hairLayer = EnsureLayer("MMDPhysicsHair");
                if (hairLayer != -1) SetLayerCollision(hairLayer, hairLayer, true);
                if (hairLayer != -1 && mmdLayer != -1) SetLayerCollision(hairLayer, mmdLayer, false);
            }

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

            // physicsGltf は unitScale 適用済みなので換算不要（=1）。
            // raw フォールバック時のみ従来の推定を使う。
            float scaleFactor = usingPhysicsGltf ? 1f : EstimateMmdScale(rigidBodies, allTransforms);
            if (usingPhysicsGltf)
                Debug.Log("[MMD Physics] physicsGltf のためスケール換算は不要です（unitScale 適用済み）。");
            else
                Debug.Log($"[MMD Physics] MMD→Unity 推定スケール: {scaleFactor:F4}");

            rigidBodyIndexToBoneRb = new Dictionary<int, Rigidbody>();
            int createdCount = 0;

            // 観測用：どのボーンに何個の剛体が乗ったか（複数乗ると1つのRigidbodyへ統合される）
            var bodiesPerBone = new Dictionary<Transform, List<string>>();

            // マスク方式用：コライダーとその group / mask を剛体の順に集める
            var maskColliders = new List<Collider>();
            var maskGroups = new List<int>();
            var maskMasks = new List<int>();

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

                if (usingPhysicsGltf && rbData.IsBoneLocal)
                {
                    // physicsGltf(space="boneLocal") は bone のローカル空間なので、
                    // 子オブジェクトの localPosition / localRotation にそのまま代入できる。
                    // モデルルートの姿勢・スケールに依存しないのが利点。
                    colObj.transform.SetParent(boneTransform, false);
                    colObj.transform.localPosition = ToVector3Local(rbData.position);
                    colObj.transform.localRotation = ToQuaternionLocal(rbData.rotation);
                }
                else
                {
                    colObj.transform.position = worldPos;
                    colObj.transform.rotation = worldRot;
                    colObj.transform.SetParent(boneTransform, true); // worldPositionStays=true でこの姿勢を保持
                }

                int layerToUse = mmdLayer;
                if (isSkirt && skirtLayer != -1) layerToUse = skirtLayer;
                else if (isHair && hairLayer != -1) layerToUse = hairLayer;
                if (layerToUse != -1) colObj.layer = layerToUse;

                // スカートは脚との衝突を有効にした関係で、恒常的な深いめり込みを避けるため
                // コライダーサイズを少し縮める。
                // スカートは体との衝突を有効にしたため、単純形状同士の重なり(めり込み)を
                // 減らすべく縮小率を強める(0.8→0.6)。髪も同様に少し縮めて突き抜けにくくする。
                float colliderScale = scaleFactor;
                if (isSkirt) colliderScale = scaleFactor * tune_skirtColliderScale;
                else if (isHair) colliderScale = scaleFactor * tune_hairColliderScale;
                AttachCollider(colObj, rbData, colliderScale, tune_contactOffsetRatio);

                var madeCol = colObj.GetComponent<Collider>();
                if (madeCol != null)
                {
                    maskColliders.Add(madeCol);
                    maskGroups.Add(Mathf.Clamp(rbData.group, 0, 15));
                    maskMasks.Add(rbData.no_collision_mask);
                }

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
                    rb.mass = Mathf.Max(rbData.mass, tune_massMin);
                    // ★Damping はPMXの値（髪は0.7前後）。慣性が1e-5オーダーの剛体には強く効き、
                    //   親の動きへの追従が遅れる原因になる。倍率で調整できる。
                    //   忠実変換off(既定)なら Bullet の秒単位定義を D=-ln(1-d) で換算してから
                    //   倍率・最小値を適用する（換算後の値に対して調整が掛かる）。
                    float srcLinDamp = tune_damperBulletFaithful ? BulletToUnityDamping(rbData.linear_damping) : rbData.linear_damping;
                    float srcAngDamp = tune_damperBulletFaithful ? BulletToUnityDamping(rbData.angular_damping) : rbData.angular_damping;
                    rb.linearDamping = Mathf.Max(srcLinDamp * tune_linearDampingScale, tune_linearDampingMin);
                    rb.angularDamping = Mathf.Max(srcAngDamp * tune_angularDampingScale, tune_angularDampingMin);
                    rb.isKinematic = isKinematicBody;
                }
                else
                {
                    float newMass = Mathf.Max(rbData.mass, tune_massMin);
                    float totalMass = rb.mass + newMass;
                    float srcLinDamp2 = tune_damperBulletFaithful ? BulletToUnityDamping(rbData.linear_damping) : rbData.linear_damping;
                    float srcAngDamp2 = tune_damperBulletFaithful ? BulletToUnityDamping(rbData.angular_damping) : rbData.angular_damping;
                    float newLinDamp = Mathf.Max(srcLinDamp2 * tune_linearDampingScale, tune_linearDampingMin);
                    float newAngDamp = Mathf.Max(srcAngDamp2 * tune_angularDampingScale, tune_angularDampingMin);
                    rb.linearDamping = (rb.linearDamping * rb.mass + newLinDamp * newMass) / totalMass;
                    rb.angularDamping = (rb.angularDamping * rb.mass + newAngDamp * newMass) / totalMass;
                    rb.mass = totalMass;
                    if (!isKinematicBody) rb.isKinematic = false; // どれか1つでも物理対象なら物理を優先
                }
                // ★アニメーションでボーンを直接動かすと、Kinematicな体のコライダーは
                //   PhysXから見て毎フレーム瞬間移動する。離散判定のままだと、速い動きで
                //   体が揺れ物の内側へ一気に入り込み、その深い侵入を強く押し返すことになる。
                //   投機的接触（ContinuousSpeculative）は速度に応じて接触の検知範囲を
                //   広げるため、深く刺さる前に穏やかに受け止められる。
                rb.collisionDetectionMode =
                    tune_collisionDetection == 2 ? CollisionDetectionMode.ContinuousDynamic :
                    tune_collisionDetection == 1 ? CollisionDetectionMode.ContinuousSpeculative :
                                                   CollisionDetectionMode.Discrete;

                // ★スリープを禁止する。
                //   Unityは動きの小さい剛体を自動で眠らせるが、眠った剛体は
                //   ジョイント経由の力では確実には起きない。揺れ物が一度眠ると
                //   ボーンにぶら下がったまま固まり、物理が効いていないように見える。
                //   MMDの揺れ物は常時わずかに動いている前提なので眠らせない。
                if (!rb.isKinematic)
                {
                    rb.sleepThreshold = 0f;
                    rb.WakeUp();
                }

                // ★めり込み解消の速度上限は、名前で選ばず動く剛体すべてに掛ける。
                //   体が速く動くと揺れ物へ深く入り込むため、その押し返しが
                //   そのまま飛び出す速度になる。ここを絞ると穏やかに戻る。
                if (!rb.isKinematic) rb.maxDepenetrationVelocity = tune_maxDepenetrationVelocity; // 衝突再有効化に伴う初期めり込みの急激な弾けを防ぐ

                rigidBodyIndexToBoneRb[i] = rb;
                createdCount++;

                List<string> namesOnBone;
                if (!bodiesPerBone.TryGetValue(boneTransform, out namesOnBone))
                {
                    namesOnBone = new List<string>();
                    bodiesPerBone[boneTransform] = namesOnBone;
                }
                namesOnBone.Add($"[{i}]{rbData.name}");
            }

            // ★慣性テンソルの倍率。
            //   質量はPMXの値のまま、回りにくさ（＝重さの手応え・遅れ）だけを増やす。
            //   重力トルクは質量×腕の長さで決まり慣性には比例しないので、慣性を上げると
            //   相対的に重力がゆっくり効くようになり「重い」動きになる。
            //   ドライブは慣性で正規化しているため、この倍率で硬さの比率は変わらない。
            //   ※データに無い調整。PhysX側の手応えを整えるためのダイヤルとして扱う。
            if (!Mathf.Approximately(tune_inertiaScale, 1f))
            {
                int scaled = 0;
                foreach (var kv in rigidBodyIndexToBoneRb)
                {
                    var rb = kv.Value;
                    if (rb == null || rb.isKinematic) continue;
                    Vector3 it = rb.inertiaTensor;
                    if (it.x <= 0f || it.y <= 0f || it.z <= 0f) continue;
                    rb.inertiaTensor = it * tune_inertiaScale;
                    scaled++;
                }
                Debug.Log($"[MMD Physics] 慣性テンソルを {tune_inertiaScale:F2} 倍にしました（{scaled} 体）。質量はデータのままです。");
            }

            Debug.Log($"[MMD Physics] 剛体 {createdCount} 個を生成しました（Rigidbodyはボーン本体に直接付与）。" +
                      (tune_damperBulletFaithful
                        ? "減衰はBullet忠実変換 D=-ln(1-d) で換算済みです（例: 0.9→2.30）。"
                        : "減衰はPMX生値の直代入です（忠実変換OFF＝旧挙動）。"));

            // ★PMXのグループ／マスクをペア単位で再現するコンポーネントを付ける。
            //   Physics.IgnoreCollision はシーンに保存されないため、再生のたびに
            //   Awake で貼り直す必要がある。
            // ★重力をMMDの単位系へ合わせる（長さを縮めた分だけ重力も縮めないと速く揺れる）
            var oldGrav = targetPrefab.GetComponent<MmdGravity>();
            if (oldGrav != null) DestroyImmediate(oldGrav);
            if (tune_matchMmdGravity)
            {
                // ★MMD本家(VMDビューア)は Bullet へ 重力 = 9.81 × GravityBaseScale(=10) を
                //   MMD単位で渡している。MMDモデルは実寸の約10倍スケール(身長≈20単位)なので、
                //   この ×10 で見た目の落下速度を合わせている。
                //   実メートルに直すと 9.81 × 10 × unitScale(0.08) = 7.85 m/s² ＝ 約0.8G。
                //   単純に unitScale 倍にすると10分の1になり、髪もスカートも垂れなくなる。
                float us = ReadUnitScale();
                float gs = us > 0f ? us * tune_gravityBaseScale : tune_gravityScale;
                var grav = targetPrefab.AddComponent<MmdGravity>();
                grav.gravityScale = gs;
                Debug.Log($"[MMD Physics] 重力の倍率を {gs:F3} にしました" +
                          (us > 0f ? $"（unitScale={us:F3} × GravityBaseScale={tune_gravityBaseScale:F0}）" : "（unitScale が読めなかったのでスライダーの値）") +
                          $" ＝ {9.81f * gs:F2} m/s²。本家(VMDビューア)がBulletへ渡している値と揃います。");
            }

            // ★再生直後の姿勢差による爆発を避けるため、始動を少し遅らせる
            var oldWarm = targetPrefab.GetComponent<MmdPhysicsWarmup>();
            if (oldWarm != null) DestroyImmediate(oldWarm);
            if (tune_warmupSeconds > 0f)
            {
                var warm = targetPrefab.AddComponent<MmdPhysicsWarmup>();
                warm.warmupSeconds = tune_warmupSeconds;
            }

            var oldMask = targetPrefab.GetComponent<MmdCollisionMask>();
            if (oldMask != null) DestroyImmediate(oldMask);
            if (tune_useCollisionMask && maskColliders.Count > 0)
            {
                var maskComp = targetPrefab.AddComponent<MmdCollisionMask>();
                maskComp.Setup(maskColliders, maskGroups, maskMasks);
                // ★スカート組は埋まり保留にしない（脚の押し上げ力を最初から通す。翻り対策で確立）
                maskComp.forceActiveNameFilter = tune_collisionForceActiveFilter ?? "";
                Debug.Log($"[MMD Physics] PMXの衝突マスクを使います（コライダー {maskColliders.Count} 個）。" +
                          (string.IsNullOrEmpty(tune_collisionForceActiveFilter)
                              ? ""
                              : $"「{tune_collisionForceActiveFilter}」を含む組は埋まり保留にしません。") +
                          "実際の適用件数は再生時にログへ出ます。");
            }
            else if (!tune_useCollisionMask)
            {
                Debug.LogWarning("[MMD Physics] 衝突は従来の3レイヤー方式です（PMXのマスクは使いません）。");
            }

            if (Time.fixedDeltaTime > 1f / 60f + 1e-5f)
                Debug.LogWarning($"[MMD Physics] Fixed Timestep が {Time.fixedDeltaTime:F4}（{1f / Time.fixedDeltaTime:F0}Hz）です。" +
                                 "MMDの物理は60fps基準で動いているため、Project Settings → Time → Fixed Timestep を 0.0167 以下" +
                                 "（髪のように細かい剛体が多い場合は 0.0083）にすると安定します。");

            // ★複数の剛体が同じボーンへ統合されると、それぞれに向けたジョイントが
            //   すべて同じ Rigidbody に集まる。別チェーンの剛体が同居していると
            //   「本来つながらないはずの親」が増え、拘束同士が競合して発散する。
            var merged = new List<string>();
            foreach (var kv in bodiesPerBone)
                if (kv.Value.Count > 1) merged.Add($"   {kv.Key.name} ← {string.Join(", ", kv.Value)}");
            if (merged.Count > 0)
            {
                merged.Sort();
                Debug.LogWarning($"[MMD Physics][観測] 同じボーンに複数の剛体が乗り、1つのRigidbodyへ統合された箇所が {merged.Count} 件あります:\n" + string.Join("\n", merged));
            }
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
            int framedFromData = 0;
            int softLimited = 0; // ★ソフトリミットを適用したスカートジョイント数
            int yawTightened = 0; // ★ヨー軸を締めた（遊び・拡大の対象から外した）スカートジョイント数

            // ★部位別ダイヤル診断用：どの剛体が何カテゴリに何件判定されたか、
            //   実際に適用されているHz/ζ倍率・遊びオフセットをコンソールで直接確認できるようにする。
            var partDialCounts = new Dictionary<PartCategory, int>();
            var partDialSamples = new List<string>();

            // 観測用：子剛体ごとに「どのジョイントでどの親につながれたか」
            var parentsOfChild = new Dictionary<Rigidbody, List<string>>();

            foreach (var jd in joints)
            {
                Rigidbody parentRb, childRb;
                if (!rbMap.TryGetValue(jd.rigid_a, out parentRb)) { skipped++; continue; }
                if (!rbMap.TryGetValue(jd.rigid_b, out childRb)) { skipped++; continue; }
                if (parentRb == null || childRb == null || parentRb == childRb) { skipped++; continue; }

                ConfigurableJoint joint = childRb.gameObject.AddComponent<ConfigurableJoint>();
                joint.connectedBody = parentRb;
                joint.enableCollision = false; // 接続相手とは衝突させない

                // ── ジョイントの基準フレーム ──
                joint.configuredInWorldSpace = false;

                if (usingPhysicsGltf && jd.rotation != null && jd.rotation.Count >= 4)
                {
                    // physicsGltf のジョイントは space="boneLocal"（refBone = 剛体Bのボーン）で
                    // position/rotation を持つ。Rigidbody はボーン本体に直付けしているので、
                    // そのローカル空間がそのまま ConfigurableJoint の基準空間になる。
                    //
                    // ここを設定しないと、角度制限が「ボーンのX/Y/Z軸」に対して適用されてしまう。
                    // MMDのジョイントは軸ごとに制限が大きく違う（縦チェーンは X:±80° / Y:±5° / Z:±10°）
                    // ため、軸の対応がずれると本来±5°しか動けない方向へ±80°動けることになり、
                    // ボーンの向きに応じて方向依存の開き方をする。
                    Quaternion jr = ToQuaternionLocal(jd.rotation);

                    // ★connectedAnchor は必ず自分で計算する。
                    //   autoConfigureConnectedAnchor の自動計算は connectedBody を代入した
                    //   時点で一度走るだけで、あとから anchor を変えても追従しない。
                    //   放置すると「anchor が 0 だった頃の connectedAnchor」が残り、
                    //   直線可動が Locked のまま拘束が初期状態で破れる。慣性の小さい髪では
                    //   その補正がそのまま角速度に化けて発散する。
                    joint.autoConfigureConnectedAnchor = false;
                    joint.anchor = ToVector3Local(jd.position);
                    joint.axis = jr * Vector3.right;
                    joint.secondaryAxis = jr * Vector3.up;
                    joint.connectedAnchor = parentRb.transform.InverseTransformPoint(
                        childRb.transform.TransformPoint(joint.anchor));
                    framedFromData++;

                    // ★Swing/Twist分離：スカート⇔下半身の「取付」ジョイントだけ、
                    //   今組んだ単一ConfigurableJointを破棄してY/X/Z単軸3連ヒンジに置き換える。
                    if (tune_decoupleSkirtSwingTwist && IsSkirtAttachmentJoint(parentRb, childRb))
                    {
                        DestroyImmediate(joint);
                        BuildDecoupledSwingTwistChain(jd, parentRb, childRb, jr, ToVector3Local(jd.position));
                        connected++;
                        List<string> dplist;
                        if (!parentsOfChild.TryGetValue(childRb, out dplist))
                        {
                            dplist = new List<string>();
                            parentsOfChild[childRb] = dplist;
                        }
                        dplist.Add($"{jd.name}→親 {parentRb.transform.name}（Swing/Twist分離）");
                        continue;
                    }
                }
                else
                {
                    // 旧挙動（raw フォールバック）：ボーンのローカル軸をそのまま使う
                    joint.autoConfigureConnectedAnchor = true;
                    joint.axis = Vector3.right;
                    joint.secondaryAxis = Vector3.up;
                }

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
                    // ★移動制限の倍率。
                    //   横リング(スカート)や房どうし(髪)を結ぶジョイントは、この移動制限で
                    //   互いを縛っている。角度の遊びをいくら増やしても、この網が伸びなければ
                    //   房は動けない（実測でも一本鎖のモミアゲ・前髪だけが反応し、
                    //   網を持つ後ろ髪とスカートは全く変化しなかった）。
                    //   エクスポーター側で lateral_slack_scale=6 が必要だったのと同じ事情。
                    //   ★スカートの横リングだけは専用倍率（既定8）：円錐に開くための円周の
                    //   伸び代で、PMX解析(必要伸び=素の5.6倍)とスイープH条件の実証値。
                    //   髪の房まで緩めないよう、全体スライダーとは分離してある。
                    bool bothSkirt = parentRb != null && childRb != null
                                  && parentRb.name.Contains("スカート") && childRb.name.Contains("スカート");
                    float linScale = bothSkirt ? Mathf.Max(1f, tune_skirtLinearScale)
                                               : Mathf.Max(1f, tune_linearSlackScale);
                    var lin = joint.linearLimit;
                    lin.limit = maxLin * linScale;
                    joint.linearLimit = lin;
                }

                // ★制限を柔らかくする。
                //   Bullet の6DOF拘束は限界付近が柔らかく、じわりと止まる。PhysX の制限は
                //   硬い壁なので、同じ数値でも到達できる角度が変わる。
                //   ばねを入れると限界を少し越えられるようになり、本家の手触りに近づく。
                if (tune_limitSoftness > 0f)
                {
                    // 柔らかさ1.0で「弱いばね」、0に近いほど硬い壁へ寄せる
                    float k = Mathf.Lerp(20000f, 200f, Mathf.Clamp01(tune_limitSoftness));
                    var ls = new SoftJointLimitSpring { spring = k, damper = k * 0.05f };
                    joint.linearLimitSpring = ls;
                    joint.angularXLimitSpring = ls;
                    joint.angularYZLimitSpring = ls;
                }

                // ── 回転可動（ラジアン→度、左右非対称対応）──
                joint.angularXMotion = MotionFor(jd.rot_min, jd.rot_max, 0);
                joint.angularYMotion = MotionFor(jd.rot_min, jd.rot_max, 1);
                joint.angularZMotion = MotionFor(jd.rot_min, jd.rot_max, 2);

                // ★MMDの角度制限は「直立の基本ポーズ」基準に作られており、かなり狭い。
                //   ダンス等で脚や腰の向きが基本ポーズと大きく変わると、重力で垂れたい
                //   スカート等がこの狭い壁に押し付けられて固まることがあるため、
                //   少し「遊び」を追加して吸収できるようにする。
                float angularSlackDeg = tune_angularSlackDeg + GetPartSlackOffsetDeg(childRb.name);

                // ★ヨー軸タイト化（VMD実測 2026-08-01）：本家スカートはターン中もヨー遅れ1〜3°で
                //   体と共回転し、遠心力をフルに受けて傾き50〜60°を出す。ヨー自由度を遊びや
                //   ソフトリミット拡大で広げると共回転の鎖が切れ、遠心力の源泉ω²rが細る。
                //   ワールド鉛直に最も近い角度軸をヨー軸として検出し、その軸だけ
                //   遊びオフセット・ソフトリミット拡大の対象から外す（PMX素の狭さを維持）。
                float slackX = angularSlackDeg, slackY = angularSlackDeg, slackZ = angularSlackDeg;
                int skirtYawAxis = -1; // 0=X, 1=Y, 2=Z（-1=対象外）
                bool skirtSoftTarget = tune_softLimitSkirt && IsSkirtSoftLimitTarget(joint, parentRb, childRb);
                if (skirtSoftTarget && tune_skirtYawTight)
                {
                    Quaternion wr = childRb.transform.rotation;
                    float dX = Mathf.Abs(Vector3.Dot((wr * joint.axis).normalized, Vector3.up));
                    float dY = Mathf.Abs(Vector3.Dot((wr * joint.secondaryAxis).normalized, Vector3.up));
                    float dZ = Mathf.Abs(Vector3.Dot((wr * Vector3.Cross(joint.axis, joint.secondaryAxis)).normalized, Vector3.up));
                    skirtYawAxis = (dX >= dY && dX >= dZ) ? 0 : (dY >= dZ ? 1 : 2);
                    if (skirtYawAxis == 0) slackX = 0f;
                    else if (skirtYawAxis == 1) slackY = 0f;
                    else slackZ = 0f;
                    yawTightened++;
                }

                // ★Z反転を掛けた基準フレームでは、X軸まわりの回転の符号も反転する。
                //   制限も同じ鏡像を通す必要があるので、min/max を入れ替えて符号を反転する。
                //   （Y/Z は下で絶対値の最大を採るため符号の影響を受けない）
                bool mirrorX = usingPhysicsGltf && tune_flipHandedness && jd.rotation != null && jd.rotation.Count >= 4;
                float rxLow = mirrorX ? -SafeGet(jd.rot_max, 0, 0f) : SafeGet(jd.rot_min, 0, 0f);
                float rxHigh = mirrorX ? -SafeGet(jd.rot_min, 0, 0f) : SafeGet(jd.rot_max, 0, 0f);

                var hx = joint.highAngularXLimit;
                hx.limit = rxHigh * Mathf.Rad2Deg + slackX;
                joint.highAngularXLimit = hx;

                var lx = joint.lowAngularXLimit;
                lx.limit = rxLow * Mathf.Rad2Deg - slackX;
                joint.lowAngularXLimit = lx;

                var ay = joint.angularYLimit;
                ay.limit = Mathf.Max(Mathf.Abs(SafeGet(jd.rot_min, 1, 0f)), Mathf.Abs(SafeGet(jd.rot_max, 1, 0f))) * Mathf.Rad2Deg + slackY;
                joint.angularYLimit = ay;

                var az = joint.angularZLimit;
                az.limit = Mathf.Max(Mathf.Abs(SafeGet(jd.rot_min, 2, 0f)), Mathf.Abs(SafeGet(jd.rot_max, 2, 0f))) * Mathf.Rad2Deg + slackZ;
                joint.angularZLimit = az;

                // ★スカートのソフトリミット焼き込み（暫定チューナーMmdWaistSoftLimitの正式版）。
                //   対象＝取付（片側だけスカート）と、tune_softLimitVertical時は縦（両側スカート・直線全ロック）。
                //   直線可動を持つ横リングは対象外。ヨー軸タイト化ON時はヨー軸のリミット拡大もスキップ
                //   （傾きの部屋だけ広げ、共回転の鎖は狭いまま維持する）。
                if (skirtSoftTarget)
                {
                    joint.angularYZLimitSpring = new SoftJointLimitSpring
                    {
                        spring = tune_softLimitSpring,
                        damper = tune_softLimitDamper,
                    };
                    if (skirtYawAxis != 1)
                    {
                        var sy = joint.angularYLimit; sy.limit *= tune_softLimitScale; joint.angularYLimit = sy;
                    }
                    if (skirtYawAxis != 2)
                    {
                        var sz = joint.angularZLimit; sz.limit *= tune_softLimitScale; joint.angularZLimit = sz;
                    }
                    softLimited++;
                }

                // ── ばね（spring_rot → 回転ドライブ）──
                ApplyRotationSpring(joint, jd.spring_rot, childRb, childRb.name);

                // ★部位別ダイヤル診断：実際にどのカテゴリに判定され、どの倍率が使われたかを記録
                if (tune_usePartDials)
                {
                    var cat = DetectPartCategory(childRb.name);
                    partDialCounts.TryGetValue(cat, out int cc);
                    partDialCounts[cat] = cc + 1;
                    if (cat != PartCategory.Other && partDialSamples.Count < 8)
                    {
                        GetPartDriveScale(childRb.name, out float hzS, out float zS);
                        float slackOff = GetPartSlackOffsetDeg(childRb.name);
                        partDialSamples.Add($"{childRb.name} → {cat} (Hz×{hzS:F2} ζ×{zS:F2} 遊び{slackOff:+0.0;-0.0}°)");
                    }
                }

                // ★投影（projection）：ソルバーが解き切れずに拘束が開いてしまったとき、
                //   剛体を強制的に引き戻すPhysXの機能。IAの髪は前後・左右の房どうしが
                //   結ばれた網目構造（閉ループ）で、慣性が 1e-5 オーダーの剛体が
                //   多数の拘束を同時に受けるため、反復回数だけでは残差が残りやすい。
                //   許容量はジョイント自身が許している遊び（linearLimit）を基準にする。
                //   遊びが 0（Locked）のものは、そのチェーンで最小の値として 1mm を使う。
                if (tune_useJointProjection)
                {
                    joint.projectionMode = JointProjectionMode.PositionAndRotation;
                    joint.projectionDistance = Mathf.Max(maxLin, 0.001f);
                    joint.projectionAngle = tune_projectionAngleDeg;
                }

                connected++;

                List<string> plist;
                if (!parentsOfChild.TryGetValue(childRb, out plist))
                {
                    plist = new List<string>();
                    parentsOfChild[childRb] = plist;
                }
                plist.Add($"{jd.name}→親 {parentRb.transform.name}");
            }

            // ★1つの子剛体が複数の親につながれている箇所を名指しする。
            //   スカートの横リング（隣どうしを環状に結ぶ）は設計どおりなので正常。
            //   髪のように本来1本鎖であるべき部位に出た場合は、剛体→ボーンの
            //   対応付けがずれて別チェーンのジョイントが同居している疑いが濃い。
            var multiParent = new List<string>();
            foreach (var kv in parentsOfChild)
                if (kv.Value.Count > 1) multiParent.Add($"   {kv.Key.transform.name}: " + string.Join(" / ", kv.Value));
            if (multiParent.Count > 0)
            {
                multiParent.Sort();
                Debug.Log($"[MMD Physics][観測] 複数の親につながれている子剛体 {multiParent.Count} 件:\n" + string.Join("\n", multiParent));
            }

            if (framedFromData > 0)
                Debug.Log($"[MMD Physics] {framedFromData} 件のジョイントで基準フレーム（anchor / axis / secondaryAxis）をデータから設定しました。");
            else if (connected > 0)
                Debug.LogWarning("[MMD Physics] ジョイントの基準フレームをデータから設定できませんでした（rawフォールバック）。角度制限はボーン軸基準になります。");

            if (tune_softLimitSkirt)
                Debug.Log($"[MMD Physics] スカートのソフトリミットを {softLimited} 本に焼き込みました" +
                          $"（ばね{tune_softLimitSpring:F0}/減衰{tune_softLimitDamper:F1}, 角度倍率×{tune_softLimitScale:F2}, " +
                          (tune_softLimitVertical ? "取付＋縦" : "取付のみ") + "）。" +
                          (tune_skirtYawTight ? $"うち {yawTightened} 本はヨー軸を検出して遊び・拡大の対象から除外しました（共回転の鎖＝遠心力の確保）。" : "") +
                          "強い押し上げの瞬間だけリミットを柔らかく超えられます（翻り対策）。");

            if (skipped > 0)
                Debug.LogWarning($"[MMD Physics] {skipped} 件のジョイントは対応する剛体が見つからずスキップしました。");

            if (tune_usePartDials)
            {
                var counts = string.Join(" / ", new[] {
                    $"スカート={GetCount(partDialCounts, PartCategory.Skirt)}",
                    $"前髪={GetCount(partDialCounts, PartCategory.Bangs)}",
                    $"もみあげ={GetCount(partDialCounts, PartCategory.Sideburns)}",
                    $"その他={GetCount(partDialCounts, PartCategory.Other)}",
                });
                Debug.Log($"[部位別ダイヤル診断] 判定件数: {counts}\n" +
                          (partDialSamples.Count > 0
                              ? "実際に適用された倍率のサンプル:\n" + string.Join("\n", partDialSamples)
                              : "★該当する剛体が1件も見つかりませんでした（名前パターンが一致していない可能性大）"));
            }

            return connected;
        }

        private static int GetCount(Dictionary<PartCategory, int> d, PartCategory k) => d.TryGetValue(k, out int v) ? v : 0;

        private void ApplyRotationSpring(ConfigurableJoint joint, List<float> springRot, Rigidbody childRb, string partName)
        {
            float sx = SafeGet(springRot, 0, 0f);
            float syz = Mathf.Max(SafeGet(springRot, 1, 0f), SafeGet(springRot, 2, 0f));

            // ★部位別ダイヤル：スカート/前髪/もみあげで戻る速さ(Hz)・減衰比(ζ)の
            //   効き方が逆を向く（2026-08-01の実ダンス数値診断で確定）ため、
            //   剛体名（jd.nameはジョイント名で剛体名と一致しないケースがあるため使わない）の
            //   パターンから倍率を引いて全身共通値に掛け合わせる。
            float partHzScale = 1f, partZetaScale = 1f;
            if (tune_usePartDials) GetPartDriveScale(partName, out partHzScale, out partZetaScale);
            float effectiveHz = tune_driveFreqHz * partHzScale;
            float effectiveZeta = tune_driveDampingRatio * partZetaScale;

            // ★慣性で正規化する。
            //   ばね定数そのものを一律に与えると、慣性が一桁以上違う髪(1.6e-5)と
            //   スカート(2.0e-4)で効き方がまるで変わる。髪が硬くなる値ではスカートが
            //   広がらず、スカートが自由に動く値では髪が発散する。
            //   物理的に意味があるのは戻ろうとする速さ ω=√(k/I) なので、
            //   k = I·ω² としてやれば全身で同じ速さに揃う。
            //   減衰も 2ζ√(k·I) = 2ζ·I·ω とすれば減衰比が揃う。
            if (tune_normalizeDriveByInertia && childRb != null)
            {
                Vector3 it = childRb.inertiaTensor;
                float inertia = (it.x + it.y + it.z) / 3f;
                if (inertia > 1e-9f)
                {
                    float omega = 2f * Mathf.PI * Mathf.Max(0f, effectiveHz);
                    float k = inertia * omega * omega;
                    float c = 2f * Mathf.Max(0f, effectiveZeta) * inertia * omega;

                    joint.rotationDriveMode = RotationDriveMode.XYAndZ;

                    var ndx = joint.angularXDrive;
                    ndx.positionSpring = Mathf.Max(sx, k);
                    ndx.positionDamper = c;
                    ndx.maximumForce = (ndx.positionSpring > 0f || c > 0f) ? Mathf.Infinity : 0f;
                    joint.angularXDrive = ndx;

                    var ndyz = joint.angularYZDrive;
                    ndyz.positionSpring = Mathf.Max(syz, k);
                    ndyz.positionDamper = c;
                    ndyz.maximumForce = (ndyz.positionSpring > 0f || c > 0f) ? Mathf.Infinity : 0f;
                    joint.angularYZDrive = ndyz;
                    return;
                }
            }

            sx = Mathf.Max(sx, tune_springRotFloor);
            syz = Mathf.Max(syz, tune_springRotFloor);

            joint.rotationDriveMode = RotationDriveMode.XYAndZ;

            // ★ダンパーに下限を設けない。
            //   ダンパーは「親に対する相対的な回転速度」に抵抗するので、
            //   慣性 1e-5 オーダーの髪に対しては 0.5 でも時定数が 1e-5 秒相当になり、
            //   相対回転が瞬時に消えて親へ溶接されたような硬さになる。
            //   重力を変えても効き方が変わらないのはこのため。
            //   MMDのジョイントはバネ定数が0で、本家はドライブを一切かけない。
            //   よってダンパーもばねに比例させ、ばね0なら0にする。
            float dampX = sx * tune_springDamperRatio;
            float dampYZ = syz * tune_springDamperRatio;

            var dx = joint.angularXDrive;
            dx.positionSpring = sx;
            dx.positionDamper = dampX;
            dx.maximumForce = (sx > 0f || dampX > 0f) ? Mathf.Infinity : 0f;
            joint.angularXDrive = dx;

            var dyz = joint.angularYZDrive;
            dyz.positionSpring = syz;
            dyz.positionDamper = dampYZ;
            dyz.maximumForce = (syz > 0f || dampYZ > 0f) ? Mathf.Infinity : 0f;
            joint.angularYZDrive = dyz;
        }

        // ★剛体名から部位カテゴリを判定する（ジョイント名(jd.name)は剛体名と一致しない
        //   ケースがあるため使わない。既存のスカート/髪判定(isSkirt/isHair)も剛体名ベース）。
        //   PMXのボーン／剛体命名慣習に合わせ、まず「前髪」「もみあげ／モミアゲ」を
        //   先にチェックしてから「スカート」を見る（"髪"を含む他の部位と混同しないため、
        //   ここでは部分一致のみを使い、"髪"全体は対象にしない＝後ろ髪等は既定の全身共通値のまま）。
        private enum PartCategory { Other, Skirt, Bangs, Sideburns }
        private PartCategory DetectPartCategory(string name)
        {
            if (string.IsNullOrEmpty(name)) return PartCategory.Other;
            if (name.Contains("前髪")) return PartCategory.Bangs;
            if (name.Contains("もみあげ") || name.Contains("モミアゲ")) return PartCategory.Sideburns;
            if (name.Contains("スカート")) return PartCategory.Skirt;
            return PartCategory.Other;
        }

        private void GetPartDriveScale(string jointName, out float hzScale, out float zetaScale)
        {
            switch (DetectPartCategory(jointName))
            {
                case PartCategory.Skirt:
                    hzScale = tune_skirtHzScale; zetaScale = tune_skirtZetaScale; return;
                case PartCategory.Bangs:
                    hzScale = tune_bangsHzScale; zetaScale = tune_bangsZetaScale; return;
                case PartCategory.Sideburns:
                    hzScale = tune_sideburnsHzScale; zetaScale = tune_sideburnsZetaScale; return;
                default:
                    hzScale = 1f; zetaScale = 1f; return;
            }
        }

        private float GetPartSlackOffsetDeg(string jointName)
        {
            if (!tune_usePartDials) return 0f;
            switch (DetectPartCategory(jointName))
            {
                case PartCategory.Skirt: return tune_skirtSlackDeg;
                case PartCategory.Bangs: return tune_bangsSlackDeg;
                case PartCategory.Sideburns: return tune_sideburnsSlackDeg;
                default: return 0f;
            }
        }

        // ★取付ジョイント(体⇔スカート)かどうかの判定。剛体名ベース(jd.nameは
        //   信頼できないため使わない、前回の部位別ダイヤルのバグ修正と同じ理由)。
        private bool IsSkirtAttachmentJoint(Rigidbody parentRb, Rigidbody childRb)
        {
            bool childSkirt = childRb != null && childRb.name.Contains("スカート");
            bool parentSkirt = parentRb != null && parentRb.name.Contains("スカート");
            return childSkirt && !parentSkirt;
        }

        // ★ソフトリミットの対象判定（JointProbe2・MmdWaistSoftLimitと同じ分類ルール）。
        //   取付＝片側だけスカート。縦＝両側スカートかつ直線が全てLocked。
        //   横リング＝両側スカートで直線可動あり → 対象外（完全に緩めると静止時に継ぎ目が開く）。
        private bool IsSkirtSoftLimitTarget(ConfigurableJoint joint, Rigidbody parentRb, Rigidbody childRb)
        {
            bool a = parentRb != null && parentRb.name.Contains("スカート");
            bool b = childRb != null && childRb.name.Contains("スカート");
            if (!a && !b) return false;
            if (a != b) return true; // 取付
            if (!tune_softLimitVertical) return false;
            return joint.xMotion == ConfigurableJointMotion.Locked
                && joint.yMotion == ConfigurableJointMotion.Locked
                && joint.zMotion == ConfigurableJointMotion.Locked; // 縦のみtrue
        }

        // ★Y軸→X軸→Z軸の単軸ヒンジ3連（軽量な中継剛体2つを挟む）で1つの取付ジョイントを置き換える。
        //   PhysXのConfigurableJointはY/Z(Swing)が円錐で結合されるため、Bulletの
        //   軸ごと独立した角度制限より実効可動域が狭くなる。各ジョイントの
        //   Twist軸(=angularXMotion)だけを使えば、その軸は他の2軸と結合されない
        //   独立したヒンジになる。合成順はMMDの回転合成順(YXZ、physics.pyの
        //   MMD_EULER_ORDER="YXZ"intrinsicと同じ)に合わせ、parent側からY→X→Zの順に繋ぐ。
        private void BuildDecoupledSwingTwistChain(JointData jd, Rigidbody parentRb, Rigidbody childRb,
                                                    Quaternion jr, Vector3 anchorLocal)
        {
            Vector3 worldAnchor = childRb.transform.TransformPoint(anchorLocal);

            // 各軸のローカル方向（childRbのrest姿勢基準）。中継体もこの瞬間の
            // childRbと同じワールド回転を持たせるので、そのままローカルaxisとして使い回せる。
            Vector3 axisX = jr * Vector3.right;
            Vector3 axisY = jr * Vector3.up;
            Vector3 axisZ = jr * Vector3.forward;

            bool mirrorX = usingPhysicsGltf && tune_flipHandedness && jd.rotation != null && jd.rotation.Count >= 4;
            float rxLow = mirrorX ? -SafeGet(jd.rot_max, 0, 0f) : SafeGet(jd.rot_min, 0, 0f);
            float rxHigh = mirrorX ? -SafeGet(jd.rot_min, 0, 0f) : SafeGet(jd.rot_max, 0, 0f);
            float ryLimit = Mathf.Max(Mathf.Abs(SafeGet(jd.rot_min, 1, 0f)), Mathf.Abs(SafeGet(jd.rot_max, 1, 0f)));
            float rzLimit = Mathf.Max(Mathf.Abs(SafeGet(jd.rot_min, 2, 0f)), Mathf.Abs(SafeGet(jd.rot_max, 2, 0f)));

            float angularSlackDeg = tune_angularSlackDeg + GetPartSlackOffsetDeg(childRb.name);
            float xLowDeg = rxLow * Mathf.Rad2Deg - angularSlackDeg;
            float xHighDeg = rxHigh * Mathf.Rad2Deg + angularSlackDeg;
            float yDeg = ryLimit * Mathf.Rad2Deg + angularSlackDeg;
            float zDeg = rzLimit * Mathf.Rad2Deg + angularSlackDeg;

            Rigidbody relayY = CreateSwingTwistRelayBody(childRb, "__relayY_" + childRb.name, worldAnchor);
            Rigidbody relayX = CreateSwingTwistRelayBody(childRb, "__relayX_" + childRb.name, worldAnchor);

            // ── 1本目: parent -> relayY（Y軸のみの単独ヒンジ）──
            var jointY = relayY.gameObject.AddComponent<ConfigurableJoint>();
            jointY.connectedBody = parentRb;
            jointY.enableCollision = false;
            jointY.configuredInWorldSpace = false;
            jointY.autoConfigureConnectedAnchor = false;
            jointY.anchor = Vector3.zero;
            jointY.axis = axisY;
            jointY.secondaryAxis = axisZ;
            jointY.connectedAnchor = parentRb.transform.InverseTransformPoint(worldAnchor);
            jointY.xMotion = jointY.yMotion = jointY.zMotion = ConfigurableJointMotion.Locked;
            jointY.angularYMotion = jointY.angularZMotion = ConfigurableJointMotion.Locked;
            jointY.angularXMotion = ConfigurableJointMotion.Limited;
            var lyLim = jointY.lowAngularXLimit; lyLim.limit = -yDeg; jointY.lowAngularXLimit = lyLim;
            var hyLim = jointY.highAngularXLimit; hyLim.limit = yDeg; jointY.highAngularXLimit = hyLim;

            // ── 2本目: relayY -> relayX（X軸のみの単独ヒンジ）──
            var jointX = relayX.gameObject.AddComponent<ConfigurableJoint>();
            jointX.connectedBody = relayY;
            jointX.enableCollision = false;
            jointX.configuredInWorldSpace = false;
            jointX.autoConfigureConnectedAnchor = false;
            jointX.anchor = Vector3.zero;
            jointX.axis = axisX;
            jointX.secondaryAxis = axisY;
            jointX.connectedAnchor = Vector3.zero; // relayY/relayXは同じワールド位置に置いてある
            jointX.xMotion = jointX.yMotion = jointX.zMotion = ConfigurableJointMotion.Locked;
            jointX.angularYMotion = jointX.angularZMotion = ConfigurableJointMotion.Locked;
            jointX.angularXMotion = ConfigurableJointMotion.Limited;
            var lxLim = jointX.lowAngularXLimit; lxLim.limit = xLowDeg; jointX.lowAngularXLimit = lxLim;
            var hxLim = jointX.highAngularXLimit; hxLim.limit = xHighDeg; jointX.highAngularXLimit = hxLim;

            // ── 3本目: relayX -> childRb（Z軸のみの単独ヒンジ、本来のジョイント位置）──
            var jointZ = childRb.gameObject.AddComponent<ConfigurableJoint>();
            jointZ.connectedBody = relayX;
            jointZ.enableCollision = false;
            jointZ.configuredInWorldSpace = false;
            jointZ.autoConfigureConnectedAnchor = false;
            jointZ.anchor = anchorLocal;
            jointZ.axis = axisZ;
            jointZ.secondaryAxis = axisX;
            jointZ.connectedAnchor = Vector3.zero;
            jointZ.xMotion = jointZ.yMotion = jointZ.zMotion = ConfigurableJointMotion.Locked;
            jointZ.angularYMotion = jointZ.angularZMotion = ConfigurableJointMotion.Locked;
            jointZ.angularXMotion = ConfigurableJointMotion.Limited;
            var lzLim = jointZ.lowAngularXLimit; lzLim.limit = -zDeg; jointZ.lowAngularXLimit = lzLim;
            var hzLim = jointZ.highAngularXLimit; hzLim.limit = zDeg; jointZ.highAngularXLimit = hzLim;

            // ばね(Hz/ζ)：spring_rot[0]=X用/[1]=Y用/[2]=Z用というPMXの並びをそのまま単軸のTwistへ渡す。
            ApplySingleAxisTwistSpring(jointY, SafeGet(jd.spring_rot, 1, 0f), relayY, childRb.name);
            ApplySingleAxisTwistSpring(jointX, SafeGet(jd.spring_rot, 0, 0f), relayX, childRb.name);
            ApplySingleAxisTwistSpring(jointZ, SafeGet(jd.spring_rot, 2, 0f), childRb, childRb.name);
        }

        private Rigidbody CreateSwingTwistRelayBody(Rigidbody template, string name, Vector3 worldPos)
        {
            var go = new GameObject(name);
            go.transform.position = worldPos;
            go.transform.rotation = template.transform.rotation; // rest姿勢基準の軸をそのまま使い回すため
            if (template.transform.parent != null)
                go.transform.SetParent(template.transform.parent, worldPositionStays: true);
            var rb = go.AddComponent<Rigidbody>();
            // 軽すぎると数値不安定、重すぎると余分な遅れが出るための折衷値。
            rb.mass = Mathf.Max(template.mass * 0.2f, 0.0001f);
            rb.useGravity = false;
            rb.linearDamping = template.linearDamping;
            rb.angularDamping = template.angularDamping;
            rb.interpolation = template.interpolation;
            rb.collisionDetectionMode = CollisionDetectionMode.Discrete;

            // ★中継剛体にはColliderを付けていないため、Unityが慣性テンソルを
            //   自動計算できず不定値になる。ばね計算(k = 慣性×ω²)がこれを
            //   直接使うため、不定な慣性のせいでばねが異常に硬くなり軸が
            //   事実上凍結する不具合が実機で発覚(取付曲げが0°付近に固着)。
            //   元剛体の慣性テンソルを質量比でスケールして明示的に与える。
            float massRatio = template.mass > 1e-9f ? (rb.mass / template.mass) : 1f;
            rb.inertiaTensor = template.inertiaTensor * massRatio;
            rb.inertiaTensorRotation = template.inertiaTensorRotation;
            return rb;
        }

        // ApplyRotationSpringの単軸(Twistのみ)版。分離後の各ジョイントに使う。
        private void ApplySingleAxisTwistSpring(ConfigurableJoint joint, float springRotValue, Rigidbody ownRb, string partName)
        {
            float partHzScale = 1f, partZetaScale = 1f;
            if (tune_usePartDials) GetPartDriveScale(partName, out partHzScale, out partZetaScale);
            float effectiveHz = tune_driveFreqHz * partHzScale;
            float effectiveZeta = tune_driveDampingRatio * partZetaScale;

            joint.rotationDriveMode = RotationDriveMode.XYAndZ;

            if (tune_normalizeDriveByInertia && ownRb != null)
            {
                Vector3 it = ownRb.inertiaTensor;
                float inertia = (it.x + it.y + it.z) / 3f;
                if (inertia > 1e-9f)
                {
                    float omega = 2f * Mathf.PI * Mathf.Max(0f, effectiveHz);
                    float k = inertia * omega * omega;
                    float c = 2f * Mathf.Max(0f, effectiveZeta) * inertia * omega;
                    var dx = joint.angularXDrive;
                    dx.positionSpring = Mathf.Max(springRotValue, k);
                    dx.positionDamper = c;
                    dx.maximumForce = (dx.positionSpring > 0f || c > 0f) ? Mathf.Infinity : 0f;
                    joint.angularXDrive = dx;
                    return;
                }
            }

            float sx = Mathf.Max(springRotValue, tune_springRotFloor);
            float dampX = sx * tune_springDamperRatio;
            var d = joint.angularXDrive;
            d.positionSpring = sx;
            d.positionDamper = dampX;
            d.maximumForce = (sx > 0f || dampX > 0f) ? Mathf.Infinity : 0f;
            joint.angularXDrive = d;
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
                //   出るため(IAモデルで実機確認済み)。テクスチャ無しの半透明(レンズ等の
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
                //   共有テクスチャのモデル(Tda式ミクV4X等)では全マテリアルにorigTextureが
                //   付き、全員がTransparent(q3000)へ昇格してしまう。透明キュー内は
                //   サブメッシュ番号順+深度書き込み(TwoPass)で描画されるため、
                //   「前髪(若い番号)の向こう側にあるメガネ(大きい番号)が深度テストに
                //   落ちて消える」症状が起きる(V4Xで実測確認)。
                //   対策：
                //   ・mask由来の昇格組(見た目ほぼ不透明) → AlphaTest帯(2452+slot)。
                //     深度を書きつつ真の半透明より先に描画されるので、透け髪や
                //     レンズ越しに正しく見える。ブレンド自体はキューに依らず有効。
                //   ・真の半透明(alphaClass=blend) → 3000+slot。MMDは材質順で
                //     描画するため、スロット順を維持して本家の重なり順を再現する。
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
                        var tex = FindTextureByIndex(md.pbrMetallicRoughness.baseColorTexture.index);
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
        private void AttachCollider(GameObject obj, RigidBodyData rbData, float scale, float contactOffsetRatio)
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

            // ★摩擦と反発をPMXの値からそのまま与える。
            //   これまで一切使っておらず、Unity既定（動摩擦0.6・静摩擦0.6・反発0）で
            //   動いていた。MMDの揺れ物は摩擦が小さく設定されていることが多く、
            //   既定のままだと体に触れた瞬間に食いついて、離れるときに弾かれる。
            var colForMat = obj.GetComponent<Collider>();
            if (colForMat != null)
            {
                var pm = new PhysicsMaterial($"mmd_{rbData.name}");
                pm.dynamicFriction = Mathf.Clamp01(rbData.friction);
                pm.staticFriction = Mathf.Clamp01(rbData.friction);
                pm.bounciness = Mathf.Clamp01(rbData.restitution);
                pm.frictionCombine = PhysicsMaterialCombine.Minimum;
                pm.bounceCombine = PhysicsMaterialCombine.Minimum;
                colForMat.material = pm;
            }

            // ★接触オフセットを「その形状自身の大きさ」から決める。
            //   PhysXの既定 contactOffset は 0.01m。MMDモデルは unitScale=0.08 で
            //   取り込まれるため、髪のカプセル（半径0.006m程度）は既定値のほうが
            //   自分の半径より大きいという逆転が起きる。すると常に接触扱いになり、
            //   慣性の小さい剛体では絶えず押し返しが入って落ち着かない。
            //   PhysXの推奨は「代表寸法の数%」なので、最小寸法の10%を用いる。
            //   マジックナンバーではなく、剛体自身のサイズから導いた値。
            var col = obj.GetComponent<Collider>();
            if (col != null)
            {
                float minDim = float.MaxValue;
                for (int k = 0; k < rbData.size.Count; k++)
                {
                    float v = Mathf.Abs(rbData.size[k]) * scale;
                    if (v > 1e-6f && v < minDim) minDim = v;
                }
                if (minDim < float.MaxValue)
                    col.contactOffset = Mathf.Clamp(minDim * contactOffsetRatio, 0.0001f, Physics.defaultContactOffset);
            }
        }

        // MMD原寸→Unityメートルのスケールを、ボーン追従剛体(mode==0)のボーン位置から推定する。
        // ═══════════════════════════════════════════
        //  観測①：再生前のセットアップ検証
        //
        //  静止状態で暴れる場合、原因のほとんどは「再生する前から
        //  拘束が破れている」か「慣性が極端に小さい」かのどちらか。
        //  どちらも Play しなくても数字で見える。
        // ═══════════════════════════════════════════
        private void ValidatePhysicsSetup()
        {
            var joints = targetPrefab.GetComponentsInChildren<ConfigurableJoint>();
            var bodies = targetPrefab.GetComponentsInChildren<Rigidbody>();

            Debug.Log($"[MMD 検証] 剛体 {bodies.Length} 体 / ジョイント {joints.Length} 個 / " +
                      $"fixedDeltaTime={Time.fixedDeltaTime:F4} ({1f / Time.fixedDeltaTime:F0}Hz) / " +
                      $"defaultSolverIterations={Physics.defaultSolverIterations} / " +
                      $"defaultSolverVelocityIterations={Physics.defaultSolverVelocityIterations}");

            // ── ① 初期のアンカーずれ（拘束が最初から破れていないか）──
            var violations = new List<KeyValuePair<float, string>>();
            int noParent = 0;
            foreach (var j in joints)
            {
                if (j.connectedBody == null) { noParent++; continue; }
                Vector3 wa = j.transform.TransformPoint(j.anchor);
                Vector3 wc = j.connectedBody.transform.TransformPoint(j.connectedAnchor);
                violations.Add(new KeyValuePair<float, string>(
                    Vector3.Distance(wa, wc),
                    $"{j.transform.name} ← {j.connectedBody.transform.name}"));
            }
            violations.Sort((x, y) => y.Key.CompareTo(x.Key));

            if (violations.Count > 0)
            {
                float sum = 0f; foreach (var v in violations) sum += v.Key;
                var sb = new System.Text.StringBuilder();
                sb.AppendLine($"[MMD 検証] アンカーずれ（0が理想）: 平均 {sum / violations.Count:F5} m / 最大 {violations[0].Key:F5} m");
                int n = Mathf.Min(8, violations.Count);
                for (int i = 0; i < n; i++) sb.AppendLine($"   {violations[i].Key:F5} m  {violations[i].Value}");
                Debug.Log(sb.ToString());
            }
            if (noParent > 0)
                Debug.LogWarning($"[MMD 検証] connectedBody が無いジョイントが {noParent} 個あります（ワールドに固定されます）。");

            // ── ② 慣性・質量（小さすぎると些細な残差で吹き飛ぶ）──
            float minInertia = float.MaxValue; string minInertiaName = "";
            float minMass = float.MaxValue; string minMassName = "";
            int noCollider = 0, kinematic = 0, scaled = 0;
            foreach (var rb in bodies)
            {
                if (rb.isKinematic) { kinematic++; continue; }
                Vector3 it = rb.inertiaTensor;
                float m = Mathf.Min(it.x, Mathf.Min(it.y, it.z));
                if (m < minInertia) { minInertia = m; minInertiaName = rb.name; }
                if (rb.mass < minMass) { minMass = rb.mass; minMassName = rb.name; }
                if (rb.GetComponentInChildren<Collider>() == null) noCollider++;
                Vector3 ls = rb.transform.lossyScale;
                if (Mathf.Abs(ls.x - ls.y) > 1e-4f || Mathf.Abs(ls.y - ls.z) > 1e-4f) scaled++;
            }
            if (minInertia < float.MaxValue)
                Debug.Log($"[MMD 検証] 最小の慣性成分 {minInertia:E3}（{minInertiaName}） / 最小質量 {minMass:F4}（{minMassName}） / " +
                          $"Kinematic {kinematic} 体 / コライダー無し {noCollider} 体");
            if (noCollider > 0)
                Debug.LogWarning($"[MMD 検証] コライダーを持たない動的剛体が {noCollider} 体あります（慣性が既定値になり挙動が不安定になります）。");
            if (scaled > 0)
                Debug.LogWarning($"[MMD 検証] 非等方スケールのボーンに乗った剛体が {scaled} 体あります（PhysXでは不安定要因です）。");

            // ── ③ ジョイントの構成（自己参照・重複）──
            int selfRef = 0; var seen = new Dictionary<Rigidbody, int>();
            foreach (var j in joints)
            {
                if (j.connectedBody != null && j.connectedBody == j.GetComponent<Rigidbody>()) selfRef++;
                var own = j.GetComponent<Rigidbody>();
                if (own != null) { int c; seen.TryGetValue(own, out c); seen[own] = c + 1; }
            }
            int multi = 0; foreach (var kv in seen) if (kv.Value > 1) multi++;
            if (selfRef > 0) Debug.LogError($"[MMD 検証] 自己参照ジョイントが {selfRef} 個あります（発散の直接原因になります）。");
            if (multi > 0) Debug.Log($"[MMD 検証] 同一剛体に複数ジョイントが付いているものが {multi} 体あります（分岐チェーンなら正常）。");
        }

        // ★Animation.updateMode の「物理に合わせる」値は Unity のバージョンで名前が違う
        //   （旧: AnimatePhysics / 新: Fixed）。どちらでも通るように名前で解決する。
        private static bool TryGetAnimationPhysicsMode(out AnimationUpdateMode mode)
        {
            if (System.Enum.TryParse("Fixed", out mode)) return true;
            if (System.Enum.TryParse("AnimatePhysics", out mode)) return true;
            mode = default(AnimationUpdateMode);
            return false;
        }

        // ★アニメーションの評価タイミングを物理に合わせる。
        //   これをしないと、体のコライダーは物理ステップの合間に瞬間移動することになり、
        //   その不連続な動きが揺れ物への衝撃として毎フレーム加わる。
        private void SyncAnimationToPhysics()
        {
            int changed = 0;

            var animators = new List<Animator>(targetPrefab.GetComponentsInChildren<Animator>(true));
            var pa = targetPrefab.GetComponentInParent<Animator>();
            if (pa != null && !animators.Contains(pa)) animators.Add(pa);
            foreach (var a in animators)
            {
                if (a.updateMode == AnimatorUpdateMode.Fixed) continue;
                Undo.RecordObject(a, "Sync Animation to Physics");
                a.updateMode = AnimatorUpdateMode.Fixed;
                EditorUtility.SetDirty(a);
                changed++;
            }

            var legacies = new List<Animation>(targetPrefab.GetComponentsInChildren<Animation>(true));
            var pl = targetPrefab.GetComponentInParent<Animation>();
            if (pl != null && !legacies.Contains(pl)) legacies.Add(pl);
            AnimationUpdateMode physicsMode;
            if (TryGetAnimationPhysicsMode(out physicsMode))
            {
                foreach (var lg in legacies)
                {
                    if (lg.updateMode == physicsMode) continue;
                    Undo.RecordObject(lg, "Sync Animation to Physics");
                    lg.updateMode = physicsMode;
                    EditorUtility.SetDirty(lg);
                    changed++;
                }
            }
            else if (legacies.Count > 0)
            {
                Debug.LogWarning("[MMD 観測] Animation の物理同期モードが見つかりませんでした。" +
                                 "Inspector の Update Mode を手動で設定してください。");
            }

            if (changed > 0)
                Debug.Log($"[MMD 観測] {changed} 個のアニメーションを物理と同じタイミング(FixedUpdate)で評価するようにしました。");
            else
                Debug.Log("[MMD 観測] すでに全て物理と同期しています。");
        }

        // ═══════════════════════════════════════════
        //  観測⑥：コライダーの向きがボーンの向きと合っているか
        //
        //  physicsGltf は glTF（右手系）で書かれている。UniGLTF は読み込み時に
        //  Z反転（位置 (x,y,-z) / 回転 (-x,-y,z,w)）してボーン階層を作るので、
        //  剛体のオフセットにも同じ変換が要る。掛け忘れると位置は微妙に、
        //  回転ははっきりとずれる。
        //
        //  ここでは「今の向き」と「Z反転を掛けた場合の向き」の両方について、
        //  ボーンの向き（親→子）との角度差を測る。後者が明らかに小さければ
        //  変換の掛け忘れが確定する。
        // ═══════════════════════════════════════════
        private void ValidateColliderOrientation()
        {
            var indices = targetPrefab.GetComponentsInChildren<MmdPhysicsImportIndex>();
            var bodies = ExtractRigidBodies();
            if (indices.Length == 0 || bodies.Count == 0)
            {
                Debug.LogWarning("[MMD 検証] 剛体が見つかりません。先に配置を実行してください。");
                return;
            }

            int n = 0;
            float sumNow = 0f, sumFlip = 0f;
            float worstNow = 0f; string worstName = "";
            var lines = new List<string>();

            foreach (var ix in indices)
            {
                int i = ix.absoluteDataIndex;
                if (i < 0 || i >= bodies.Count) continue;
                var d = bodies[i];
                if (d.shape != 2) continue;                       // カプセルだけを見る
                if (d.rotation == null || d.rotation.Count < 4) continue;

                Transform bone = ix.transform.parent;
                if (bone == null) continue;

                // ボーンの向き＝最初の子ボーンへの方向（子が無ければ自分のY軸）
                Transform tail = null;
                for (int k = 0; k < bone.childCount; k++)
                {
                    var c = bone.GetChild(k);
                    if (c.GetComponent<MmdPhysicsImportIndex>() != null) continue; // コライダーの入れ物は除く
                    tail = c; break;
                }
                Vector3 boneDir = tail != null
                    ? (tail.position - bone.position).normalized
                    : bone.up;
                if (boneDir.sqrMagnitude < 1e-8f) continue;

                // 今の向き（そのまま代入した結果）
                Vector3 dirNow = ix.transform.up;

                // Z反転を掛けた場合の向き
                var q = d.rotation;
                Quaternion flipped = tune_flipHandedness
                    ? new Quaternion(q[0], q[1], q[2], q[3])       // 既に反転済みなので「反転しない場合」を比較対象にする
                    : new Quaternion(-q[0], -q[1], q[2], q[3]);
                Vector3 dirFlip = bone.TransformDirection(flipped * Vector3.up);

                float aNow = Vector3.Angle(dirNow, boneDir);
                float aFlip = Vector3.Angle(dirFlip, boneDir);
                if (aNow > 90f) aNow = 180f - aNow;   // 長軸は向きの反転を区別しない
                if (aFlip > 90f) aFlip = 180f - aFlip;

                n++; sumNow += aNow; sumFlip += aFlip;
                if (aNow > worstNow) { worstNow = aNow; worstName = d.name; }
                if (lines.Count < 12) lines.Add($"   {d.name}: 今 {aNow:F1}° / 反転を切ると {aFlip:F1}°");
            }

            if (n == 0)
            {
                Debug.Log("[MMD 検証] 対象となるカプセル剛体が見つかりませんでした。");
                return;
            }

            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"[MMD 検証] コライダーの長軸とボーンの向きの角度差（カプセル {n} 個）:");
            sb.AppendLine($"   今の実装      平均 {sumNow / n:F1}° / 最大 {worstNow:F1}°（{worstName}）");
            sb.AppendLine($"   {(tune_flipHandedness ? "Z反転しない場合" : "Z反転した場合")} 平均 {sumFlip / n:F1}°");
            sb.AppendLine(tune_flipHandedness
                ? "   ※Z反転は適用済みです。「今の実装」が小さければ正しく効いています。"
                : "   ※Z反転側が明らかに小さければ、座標系変換の掛け忘れです。");
            sb.AppendLine("   ※どちらも同程度に大きければ、データがそう作られています。");
            foreach (var l in lines) sb.AppendLine(l);
            Debug.Log(sb.ToString());
        }

        // ═══════════════════════════════════════════
        //  観測⑤：アニメーションと物理が同じボーンを取り合っていないか
        //
        //  Rigidbody直付け方式では、物理が動かすのも Animator が動かすのも
        //  同じ Transform になる。アニメーションクリップに揺れ物ボーンの
        //  カーブが含まれていると、毎フレーム物理の結果が上書きされ、
        //  その差分が次のフレームで速度として現れて暴れる。
        //  ベイク済みの物理をクリップに含めて書き出した場合に起きやすい。
        // ═══════════════════════════════════════════
        private void ValidateAnimationConflict()
        {
            // クリップは Animator / 旧 Animation のどちらにも載りうるし、
            // モデルの親側に付いていることもある。両方・上下方向とも探す。
            var clips = new List<AnimationClip>();
            Transform root = null;

            var animators = new List<Animator>(targetPrefab.GetComponentsInChildren<Animator>(true));
            var parentAnimator = targetPrefab.GetComponentInParent<Animator>();
            if (parentAnimator != null && !animators.Contains(parentAnimator)) animators.Add(parentAnimator);

            foreach (var a in animators)
            {
                if (a.runtimeAnimatorController == null) continue;
                foreach (var c in a.runtimeAnimatorController.animationClips)
                    if (c != null && !clips.Contains(c)) clips.Add(c);
                if (root == null) root = a.transform;
            }

            var legacies = new List<Animation>(targetPrefab.GetComponentsInChildren<Animation>(true));
            var parentLegacy = targetPrefab.GetComponentInParent<Animation>();
            if (parentLegacy != null && !legacies.Contains(parentLegacy)) legacies.Add(parentLegacy);

            foreach (var lg in legacies)
            {
                foreach (AnimationState st in lg)
                    if (st.clip != null && !clips.Contains(st.clip)) clips.Add(st.clip);
                if (lg.clip != null && !clips.Contains(lg.clip)) clips.Add(lg.clip);
                if (root == null) root = lg.transform;
            }

            Debug.Log($"[MMD 検証] Animator {animators.Count} 個 / Animation {legacies.Count} 個 / クリップ {clips.Count} 本を見つけました。");

            // ★評価タイミングの確認。
            //   既定では描画フレーム(Updateループ)で評価されるため、物理(FixedUpdate)とズレる。
            //   物理120Hz・描画60fpsなら、体のコライダーは1ステップ静止→次で瞬間移動を繰り返す。
            //   揺れ物はその不連続な動きを毎回受けるので暴れる。
            foreach (var a2 in animators)
            {
                bool ok = a2.updateMode == AnimatorUpdateMode.Fixed;
                string msg = $"[MMD 検証] Animator '{a2.name}' の Update Mode = {a2.updateMode}";
                if (ok) Debug.Log(msg + "（物理と同期しています）");
                else Debug.LogWarning(msg + " ★物理とズレています。「アニメーションを物理と同期させる」を押してください。");
            }
            foreach (var lg2 in legacies)
            {
                AnimationUpdateMode want;
                bool ok = TryGetAnimationPhysicsMode(out want) && lg2.updateMode == want;
                string msg = $"[MMD 検証] Animation '{lg2.name}' の Update Mode = {lg2.updateMode}";
                if (ok) Debug.Log(msg + "（物理と同期しています）");
                else Debug.LogWarning(msg + " ★物理とズレています。「アニメーションを物理と同期させる」を押してください。");
            }

            if (clips.Count == 0)
            {
                Debug.LogWarning("[MMD 検証] アニメーションクリップが見つかりませんでした。" +
                                 "Timeline(PlayableDirector)や外部スクリプトで再生している場合は、" +
                                 "そのクリップに揺れ物ボーンのカーブが含まれていないか手で確認してください。");
                return;
            }

            if (root == null) root = targetPrefab.transform;

            foreach (var clip in clips)
            {
                var conflicting = new SortedSet<string>();
                int curves = 0;

                foreach (var b in AnimationUtility.GetCurveBindings(clip))
                {
                    if (b.type != typeof(Transform)) continue;
                    curves++;
                    Transform t = string.IsNullOrEmpty(b.path) ? root : root.Find(b.path);
                    if (t == null && targetPrefab != null && !string.IsNullOrEmpty(b.path))
                        t = targetPrefab.transform.Find(b.path);
                    if (t == null)
                    {
                        // パスで引けない場合は末尾の名前で探す（親子関係の差を吸収）
                        int slash = b.path.LastIndexOf('/');
                        string leaf = slash >= 0 ? b.path.Substring(slash + 1) : b.path;
                        foreach (var cand in targetPrefab.GetComponentsInChildren<Transform>(true))
                            if (cand.name == leaf) { t = cand; break; }
                    }
                    if (t == null) continue;
                    var rb = t.GetComponent<Rigidbody>();
                    if (rb != null && !rb.isKinematic) conflicting.Add(t.name);
                }

                if (conflicting.Count == 0)
                {
                    Debug.Log($"[MMD 検証] クリップ '{clip.name}': 物理で動く剛体のボーンにカーブはありません（競合なし / Transformカーブ {curves} 本）。");
                }
                else
                {
                    var names = new List<string>(conflicting);
                    int show = Mathf.Min(12, names.Count);
                    var sb = new System.Text.StringBuilder();
                    sb.AppendLine($"[MMD 検証] クリップ '{clip.name}': 物理で動く剛体と同じボーンを {conflicting.Count} 本のカーブが動かしています。");
                    for (int i = 0; i < show; i++) sb.AppendLine("   " + names[i]);
                    if (names.Count > show) sb.AppendLine($"   … 他 {names.Count - show} 本");
                    sb.AppendLine("   ※アニメーションが毎フレーム物理の結果を上書きするため、その差分が速度になって暴れます。");
                    sb.AppendLine("   ※対処：揺れ物のカーブを含まないモーションを使うか、これらのボーンを物理の対象から外してください。");
                    Debug.LogError(sb.ToString());
                }
            }
        }

        // ═══════════════════════════════════════════
        //  観測④：静止姿勢での食い込みを計測する
        //
        //  スカートが膨らむ場合、原因は「静止姿勢ですでに食い込んでいて、
        //  再生開始と同時に押し出される」ことが多い。モデルの静止姿勢は
        //  作者が正しいと考えた形なので、そこで押し出しが起きているなら
        //  当たり判定側が実物より太いことになる。
        //  PMXのマスクで実際に衝突する組だけを対象に、Physics.ComputePenetration
        //  で重なりの深さを測る。
        // ═══════════════════════════════════════════
        private void ValidateRestOverlap()
        {
            var bodies = ExtractRigidBodies();
            var indices = targetPrefab.GetComponentsInChildren<MmdPhysicsImportIndex>();

            var colByIndex = new Dictionary<int, Collider>();
            foreach (var ix in indices)
            {
                var c = ix.GetComponent<Collider>();
                if (c != null) colByIndex[ix.absoluteDataIndex] = c;
            }

            if (bodies.Count == 0 || colByIndex.Count == 0)
            {
                Debug.LogWarning("[MMD 検証] 剛体またはコライダーが見つかりません。先に配置を実行してください。");
                return;
            }

            // ★ジョイントで繋がれている剛体どうしは衝突しない（enableCollision=false）。
            //   これはRigidbody単位で効くので、同じRigidbodyに統合されたコライダーも
            //   まとめて対象外になる。除外しないと本題（本当に押し出される組）が埋もれる。
            //   参照そのものを鍵にする（インスタンスIDはUnityのバージョンで扱いが変わるため使わない）
            var linked = new Dictionary<Rigidbody, HashSet<Rigidbody>>();
            foreach (var j in targetPrefab.GetComponentsInChildren<ConfigurableJoint>())
            {
                if (j.enableCollision) continue;
                var own = j.GetComponent<Rigidbody>();
                if (own == null || j.connectedBody == null) continue;

                HashSet<Rigidbody> set;
                if (!linked.TryGetValue(own, out set)) { set = new HashSet<Rigidbody>(); linked[own] = set; }
                set.Add(j.connectedBody);

                if (!linked.TryGetValue(j.connectedBody, out set)) { set = new HashSet<Rigidbody>(); linked[j.connectedBody] = set; }
                set.Add(own);
            }

            var overlaps = new List<KeyValuePair<float, string>>();
            int pairs = 0;
            int skippedLinked = 0;

            for (int i = 0; i < bodies.Count; i++)
            {
                Collider ca;
                if (!colByIndex.TryGetValue(i, out ca) || ca == null) continue;

                for (int j = i + 1; j < bodies.Count; j++)
                {
                    Collider cb;
                    if (!colByIndex.TryGetValue(j, out cb) || cb == null) continue;

                    // PMXのマスク判定（双方向）
                    bool collide = ((bodies[j].no_collision_mask & (1 << Mathf.Clamp(bodies[i].group, 0, 15))) != 0)
                                && ((bodies[i].no_collision_mask & (1 << Mathf.Clamp(bodies[j].group, 0, 15))) != 0);
                    if (!collide) continue;

                    // ★動かないもの同士は除外する。
                    //   mode=0（ボーン追従＝Kinematic）同士は重なっていても誰も押し出されない。
                    //   MMDでは体に複数の当たり判定を重ねて置くのが普通なので、
                    //   これを数えると本題（揺れ物が押し出されているか）が埋もれる。
                    if (bodies[i].mode == 0 && bodies[j].mode == 0) continue;

                    // 同じRigidbodyにぶら下がるコライダー同士もPhysXは衝突させない
                    var rbi = ca.attachedRigidbody;
                    var rbj = cb.attachedRigidbody;
                    if (rbi != null && rbi == rbj) continue;

                    if (rbi != null && rbj != null)
                    {
                        HashSet<Rigidbody> set;
                        if (linked.TryGetValue(rbi, out set) && set.Contains(rbj))
                        {
                            skippedLinked++;
                            continue;
                        }
                    }

                    pairs++;

                    Vector3 dir; float dist;
                    if (Physics.ComputePenetration(
                            ca, ca.transform.position, ca.transform.rotation,
                            cb, cb.transform.position, cb.transform.rotation,
                            out dir, out dist) && dist > 1e-5f)
                    {
                        overlaps.Add(new KeyValuePair<float, string>(
                            dist, $"{bodies[i].name}(mode{bodies[i].mode}) ⇔ {bodies[j].name}(mode{bodies[j].mode})"));
                    }
                }
            }

            overlaps.Sort((x, y) => y.Key.CompareTo(x.Key));

            if (overlaps.Count == 0)
            {
                Debug.Log($"[MMD 検証] 静止姿勢で食い込んでいる組はありません（動く剛体が関わる衝突対象 {pairs} 組を確認、ジョイント接続 {skippedLinked} 組は除外）。" +
                          "膨らみの原因は接触ではなく、ジョイントの遊びの側にあります。");
                return;
            }

            float sum = 0f; foreach (var o in overlaps) sum += o.Key;
            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"[MMD 検証] 静止姿勢で食い込んでいる組: {overlaps.Count} / {pairs} 組" +
                          $"（動かないもの同士とジョイント接続 {skippedLinked} 組は除外済み）　" +
                          $"平均 {sum / overlaps.Count:F4} m / 最大 {overlaps[0].Key:F4} m");
            int n = Mathf.Min(12, overlaps.Count);
            for (int k = 0; k < n; k++) sb.AppendLine($"   {overlaps[k].Key:F4} m  {overlaps[k].Value}");
            sb.AppendLine("   ※再生開始と同時に、この深さぶん外へ押し出されます。");
            Debug.LogWarning(sb.ToString());
        }

        // ═══════════════════════════════════════════
        //  観測②：実行時ウォッチャーの付与 / 除去
        // ═══════════════════════════════════════════
        // 揺れの計測コンポーネントを付け外しする
        private void ToggleMotionStats()
        {
            var existing = targetPrefab.GetComponent<MmdMotionStats>();
            if (existing != null)
            {
                Undo.DestroyObjectImmediate(existing);
                Debug.Log("[MMD 計測] 揺れの計測を除去しました。");
                return;
            }
            Undo.AddComponent<MmdMotionStats>(targetPrefab);
            Debug.Log("[MMD 計測] 揺れの計測を付与しました。この状態で Play すると、" +
                      "20秒後に部位ごとの曲がり量と遅れが本家の目標値と並べて出ます。");
        }

        private void ToggleRuntimeWatcher()
        {
            var existing = targetPrefab.GetComponent<MmdPhysicsWatcher>();
            if (existing != null)
            {
                Undo.DestroyObjectImmediate(existing);
                Debug.Log("[MMD 観測] 実行時ウォッチャーを除去しました。");
                return;
            }

            var w = Undo.AddComponent<MmdPhysicsWatcher>(targetPrefab);
            w.moveThreshold = watch_moveThreshold;
            w.pauseOnFirst = watch_pauseOnFirst;
            Debug.Log($"[MMD 観測] 実行時ウォッチャーを付与しました（移動量しきい値 {watch_moveThreshold:F2} m / " +
                      $"最初の発散で一時停止 {(watch_pauseOnFirst ? "する" : "しない")}）。この状態で Play してください。");
        }

        // ★glTF(右手系Y-up) → Unity(左手系Y-up) の変換。UniGLTF がボーン階層に対して
        //   行っているのと同じ Z 反転を、剛体・ジョイントのオフセットにも掛ける。
        //   位置は (x, y, -z)、回転は (-x, -y, z, w)。
        //   PMX 自体は Unity と同じ左手系なので、PMX→glTF→Unity で2回反転して元に戻る。
        private Vector3 ToVector3Local(List<float> v)
        {
            if (v == null || v.Count < 3) return Vector3.zero;
            return tune_flipHandedness
                ? new Vector3(v[0], v[1], -v[2])
                : new Vector3(v[0], v[1], v[2]);
        }

        private Quaternion ToQuaternionLocal(List<float> q)
        {
            if (q == null || q.Count < 4) return Quaternion.identity;
            var r = tune_flipHandedness
                ? new Quaternion(-q[0], -q[1], q[2], q[3])
                : new Quaternion(q[0], q[1], q[2], q[3]);
            return (r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w) > 1e-8f ? Quaternion.Normalize(r) : Quaternion.identity;
        }

        private static Vector3 ToVector3(List<float> v)
        {
            if (v == null || v.Count < 3) return Vector3.zero;
            return new Vector3(v[0], v[1], v[2]);
        }

        private static Quaternion ToQuaternion(List<float> q)
        {
            if (q == null || q.Count < 4) return Quaternion.identity;
            var r = new Quaternion(q[0], q[1], q[2], q[3]);
            // 正規化されていない値が来ても崩れないように保険をかける
            return (r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w) > 1e-8f ? Quaternion.Normalize(r) : Quaternion.identity;
        }

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

        // ═══════════════════════════════════════════
        //  観測③：ジョイントの接続関係をデータそのものから一覧する
        //
        //  「本来つながらないはずの親」が本当にデータ由来なのかを確かめる。
        //  MMDの慣習ではジョイント名は「子側の剛体名」と一致するため、
        //  一致率が低ければ rigidA/rigidB の番号がずれている疑いが濃い。
        //  さらに raw 版と physicsGltf 版の接続番号を突き合わせ、
        //  エクスポーター側で食い違っていないかも確認する。
        // ═══════════════════════════════════════════
        private void DumpJointConnections(string keyword)
        {
            var bodies = ExtractRigidBodies();      // ここで usingPhysicsGltf が決まる
            bool pgMode = usingPhysicsGltf;
            var joints = ExtractJoints();

            if (bodies.Count == 0 || joints.Count == 0)
            {
                Debug.LogWarning("[MMD 観測] 剛体またはジョイントが読めませんでした。");
                return;
            }

            string NameOf(int i) => (i >= 0 && i < bodies.Count) ? bodies[i].name : "?";

            int mismatch = 0, checkedCount = 0;
            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"[MMD 観測] ジョイント接続一覧（'{keyword}' を含むもの）:");

            foreach (var jd in joints)
            {
                string an = NameOf(jd.rigid_a);
                string bn = NameOf(jd.rigid_b);

                // MMDの慣習：ジョイント名 == 子側(rigidB)の剛体名
                if (!string.IsNullOrEmpty(jd.name) && bn != "?")
                {
                    checkedCount++;
                    if (jd.name != bn) mismatch++;
                }

                bool hit = string.IsNullOrEmpty(keyword)
                           || (jd.name != null && jd.name.Contains(keyword))
                           || an.Contains(keyword) || bn.Contains(keyword);
                if (hit)
                {
                    string mark = (!string.IsNullOrEmpty(jd.name) && jd.name != bn) ? " ★名前不一致" : "";
                    sb.AppendLine($"   {jd.name}: [{jd.rigid_a}]{an} → [{jd.rigid_b}]{bn}{mark}");
                }
            }
            Debug.Log(sb.ToString());
            Debug.Log($"[MMD 観測] ジョイント名と子剛体名の不一致: {mismatch} / {checkedCount} 件" +
                      (mismatch > checkedCount / 4
                         ? "　★不一致が多すぎます。rigidA/rigidB の番号がずれている可能性があります。"
                         : "　（少数なら命名の揺れなので正常）"));

            // ── raw 版と physicsGltf 版で接続番号が一致するかを突き合わせる ──
            if (pgMode)
            {
                bool saved = usingPhysicsGltf;
                usingPhysicsGltf = false;
                var rawJoints = ExtractJoints();
                usingPhysicsGltf = saved;

                if (rawJoints.Count == joints.Count)
                {
                    int diff = 0;
                    var diffSb = new System.Text.StringBuilder();
                    for (int i = 0; i < joints.Count; i++)
                    {
                        if (joints[i].rigid_a != rawJoints[i].rigid_a || joints[i].rigid_b != rawJoints[i].rigid_b)
                        {
                            diff++;
                            if (diff <= 8)
                                diffSb.AppendLine($"   [{i}] {joints[i].name}: physicsGltf({joints[i].rigid_a}→{joints[i].rigid_b}) vs raw({rawJoints[i].rigid_a}→{rawJoints[i].rigid_b})");
                        }
                    }
                    if (diff > 0)
                        Debug.LogError($"[MMD 観測] raw と physicsGltf で接続番号が食い違うジョイントが {diff} 件あります（エクスポーター側の不整合）:\n" + diffSb);
                    else
                        Debug.Log("[MMD 観測] raw と physicsGltf の接続番号は全件一致しています（エクスポーター側は整合）。");
                }
                else
                {
                    Debug.LogWarning($"[MMD 観測] ジョイント件数が raw={rawJoints.Count} / physicsGltf={joints.Count} で一致しません。");
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