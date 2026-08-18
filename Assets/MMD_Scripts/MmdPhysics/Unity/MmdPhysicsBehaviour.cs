// ===========================================================================
// Unity Bullet 互換物理エンジン – Unity ブリッジ
// PMX を読み込み、物理ワールドを毎フレーム進め、ボーン Transform と同期する。
// エンジンは PMX ネイティブ座標 (左手系 Y-up) で動作し、境界で Unity 座標へ変換。
// ===========================================================================

using System.Collections.Generic;
using UnityEngine;
using BulletPhysics.Pmx;

namespace BulletPhysics.Unity
{
    /// <summary>MMD/PMX 物理を Unity 上で駆動するコンポーネント。</summary>
    public sealed class MmdPhysicsBehaviour : MonoBehaviour
    {
        public enum InputSource { Pmx, Glb }

        // ★既定 = Glb (2026-08-10)。Unity 上の運用は必ず GLB 経由 (インポーターが
        //   Source=Glb / GlbPath を設定する) であり、Pmx 直読みはヘッドレス検証用の経路。
        //   既定が Pmx だと、手で AddComponent したときだけ既定が実運用と食い違っていた。
        //   ※ヘッドレスのハーネス (tools/*) は PmxReader を直接使うため、この既定に依存しない。
        [Tooltip("入力: Glb=GLBのextras.mmd経由(Unityでの通常運用) / Pmx=PMX直読み(検証用)。どちらも同一の物理駆動")]
        public InputSource Source = InputSource.Glb;

        [Tooltip("読み込む .pmx ファイルの絶対 or Assets 相対パス (Source=Pmx のとき)")]
        public string PmxPath = "";

        [Tooltip("読み込む .glb ファイルのパス (Source=Glb のとき。extras.mmd から剛体/Joint/ボーンを構築)")]
        public string GlbPath = "";

        [Tooltip("剛体 BoneIndex -> ボーン Transform のマップ (ボーン名で解決)。GLBは import 済みスケルトンのルート")]
        public Transform ModelRoot;

        [Tooltip("エンジン(PMXネイティブ単位) -> Unity 配置スケール。Unity側モデルが縮小配置される運用向け")]
        public float UnitScale = 0.08f;

        [Header("Solver")]
        public float Gravity = 98f;         // MMD スケール重力 (約 9.8 * 10)
        public int SolverIterations = 10;
        // ★既定 = 1/60 × SubSteps 1 (2026-08-09 ジャダー対策で 1/30×2サブ から変更)。
        //   実効刻みは 1/60 で従来と同一 → MMD忠実度は数値まで一致することをヘッドレスで確認済み
        //   (bonecheck 傾き中央11.20 / p90 23.52 / 12窓比1.0611 が変更前と同値)。CPUも同等。
        //   利点: Time.fixedDeltaTime(=1/60, 下の AlignUnityFixedTimestep が自動整列) と一致するため
        //   毎FixedUpdateでちょうど1ステップ進み、更新間隔が等間隔になる(髪/スカートのコマ落ちが消える)。
        //   従来の 1/30 では 1FixedUpdate あたりの内部ステップが 0,1,0,1,1,... と変動し
        //   実時間の更新間隔が 20ms/40ms とバラついていた。詳細は DESIGN.md「コマ落ち(ジャダー)」節。
        // ★2026-08-13: SubSteps を 1 → 2 (実効 1/60 → 1/120)。貫入対策で、忠実度も同時に改善する。
        //   理由と実測は PhysicsWorld.SubSteps の注記を参照 (あちらは 1/30×4 で同じ実効刻み)。
        //   ★FixedTimeStep は 1/60 のまま触らないこと。ここを下げると Time.fixedDeltaTime との
        //   整列 (下の AlignUnityFixedTimestep) が崩れ、コマ落ち(ジャダー)が戻る。細刻み化は SubSteps で行う。
        public int SubSteps = 2;
        public float FixedTimeStep = 1f / 60f;

        [Header("Smoothness (コマ落ち/ジャダー対策)")]
        // 症状: 髪やスカートがカクついて見える。原因は「物理の更新間隔が実時間で不均一」なこと。
        //   Unity の FixedUpdate は Time.fixedDeltaTime 間隔(既定0.02s=50Hz)で呼ばれるが、
        //   エンジンは FixedTimeStep(既定1/30) のアキュムレータなので、内部ステップは
        //   実時間 20ms / 40ms とバラバラな間隔でしか進まない (実測: 0,1,0,1,1,... の周期)。
        //   物理は毎回33.3ms分進むのに表示間隔が揃わない=ジャダー。
        // 対策: Time.fixedDeltaTime と FixedTimeStep を一致させ、毎FixedUpdateでちょうど1ステップ進める。
        //   FixedTimeStep=1/60・SubSteps=1 は実効刻みが現行(1/30×2サブ)と同一のため、
        //   ヘッドレス検証でMMD忠実度が完全一致することを確認済み (傾き11.20/p90 23.52/12窓比1.0611)。CPUも同等。
        // ★既定ON (2026-08-09)。Unity全体の物理刻み(Time.fixedDeltaTime)を FixedTimeStep に合わせる。
        //   既定 0.02(50Hz) → 1/60(60Hz) になる。Custom運用では PhysX はパーク済みなので実害はない。
        //   他のFixedUpdate処理も60Hzになる点だけ留意 (呼び出し回数が2割増)。OFFにすると未整列時に警告のみ。
        [Tooltip("ONで Time.fixedDeltaTime を FixedTimeStep に合わせる (毎FixedUpdate=1ステップ=等間隔)。Unity全体の物理刻みを変える点に注意")]
        public bool AlignUnityFixedTimestep = true;

        [Header("Jitter (静止時の細かい振動)")]
        // ★静止しているのに揺れ物が細かく震える件の対策 (2026-08-10 調査)。
        //   原因: 拘束の位置誤差(Baumgarte)を「実速度」として打ち消しているため、毎ステップ
        //   運動エネルギーが供給され続け、静止状態に落ち着かない。
        //   split impulse は位置補正を擬似速度側へ分離し、実速度を汚さない標準的な対策。
        //   実測(IA・静止10秒後・動的剛体の残留運動の平均):
        //     既定           |v|0.793 |w|1.367
        //     ジョイントのみ |v|0.690 |w|1.188
        //     接触のみ       |v|0.714 |w|1.314
        //     両方ON         |v|0.576 |w|0.943   ← 約3割減
        //   ※ソルバ反復を10→40にしても改善しない(むしろ微増)=収束不足ではない。
        //   既定OFF=従来の挙動のまま(Bullet 2.75の接触も非split)。見た目をA/Bして決めること。
        [Tooltip("ジョイントの位置補正を擬似速度へ分離する。静止時のジッタが減る。既定OFF=従来挙動")]
        public bool JointSplitImpulse = false;
        [Tooltip("接触の位置補正を擬似速度へ分離する。既定OFF=Bullet 2.75準拠")]
        public bool ContactSplitImpulse = false;

        // Bullet のスリープ(非活性化)。静止した剛体を計算から外す。MMDは有効
        // (ユーザー実機でMMDのIAの序盤=静止ポーズ中に髪の揺れが止まることを確認)。
        // ★既定OFF: 実装済みだが現状ほとんど発動しない。当エンジンの静止時の残留運動が
        //   Bullet のしきい値を超えているため (IA |w|平均1.5 > しきい値1.0、101体中2体しか眠らない)。
        //   残留を下げるのが先。しきい値を緩めれば眠るが、動くべき揺れ物が固まる危険がある。
        [Tooltip("静止した剛体を非活性化して計算から外す(Bullet相当)。現状ほとんど発動しないため既定OFF")]
        public bool EnableSleeping = false;

        [Header("Startup")]
        // 起動直後、アニメがフレーム0姿勢を確定させた後に物理をボーンへ再整合する遅延(フレーム数)。
        // バインド姿勢→フレーム0への瞬間移動でスカート等が脚へ貫入(突き抜け)するのを防ぐ。
        // Animator は Update と LateUpdate の間で姿勢を書くため、LateUpdate 時点ではフレーム0が
        // 反映されている。そこで FK-rest リセット(ResetBodiesToBonePoseFk 相当)を掛けると、
        // バインド位置に取り残された動的剛体(スカート/髪)が posed 骨格の周りへ置き直される。
        // 0=無効(従来どおり Start 時のバインド基準のみ) / 1=フレーム0適用後の最初のLateUpdate /
        // 2以上=さらに数フレーム保持してからライブ物理へ渡す(取りこぼし保険)。
        [Tooltip("起動直後にアニメのフレーム0姿勢へ物理を再整合する遅延フレーム数。バインド→フレーム0の瞬間移動による貫入(突き抜け)対策。0で無効。")]
        public int PoseResetDelayFrames = 2;

        // アニメがループ/巻き戻し/シークすると、最終フレーム→先頭でポーズが不連続に飛ぶ。
        // その差分がそのまま駆動剛体(BoneFollow)の速度になり、体のコライダーが高速で
        // 揺れ物を薙ぎ払う (UE5移植版の実測: 左腕が1フレームで31.6cm移動 → しっぽが一過性で3.25倍)。
        // 駆動ボーンの目標が1フレームでこの距離以上飛んだら、起動時と同じ再整合をやり直す。
        // ★必ず起動時と同じ経路 (PoseResetDelayFrames のカウンタを張り直す) を通すこと。
        //   「その場で1回 reset して、そのフレームの物理を捨てる」実装は悪化すると実測済み。
        // 単位は PMX ネイティブ (1 ≒ 8cm)。既定 3 = 24cm/フレーム。
        // ★この距離だけでは足りない: 速いダンスの手首は単独で 24.3cm/フレーム 動く実測がある。
        //   同時に飛んだ本数の条件 (TeleportResetMinFraction) と必ず併用すること。0 で無効。
        [Tooltip("駆動ボーンが1フレームでこの距離(PMX単位, 3≒24cm)以上飛んだらテレポートとみなし、起動時と同じ再整合をやり直す。アニメのループ境界対策。0で無効。")]
        public float TeleportResetThreshold = 3f;

        // ★1本だけ飛ぶのは速い腕/手首であってテレポートではない。骨格ごと飛んだときだけ
        //   再整合したいので、駆動ボーンのうちこの割合以上が同時に飛んだことを条件にする。
        //   実測 (モデルM・7001フレームのダンスを2周): ループ境界は 24本中10本が同時に飛ぶ (最大48cm)。
        //   一方、ダンス中の 右手首 単独の 24.3cm は 24本中1本 → 距離だけの判定では誤検出していた。
        [Tooltip("テレポートと判定するのに必要な「同時に飛んだ駆動ボーン」の割合。1本だけ速い腕/手首を誤検出しないための条件。")]
        [Range(0f, 1f)] public float TeleportResetMinFraction = 0.25f;

        // ★PMX mode2 (物理演算+ボーン位置合わせ) の実装 (2026-08-10)。
        //   mode2 剛体は「位置はボーン階層から、回転は物理から」がMMDの仕様。従来これが未実装で
        //   mode1 と同じ完全自由になっていたため、スカートがMMDより柔らかかった
        //   (Tda式初音ミクV4X はスカート66個中32個が mode2。実測で揺れ幅 0.257→0.188)。
        //   既定ON。mode2 剛体を持たないモデル(IA等)では何もしないので無影響。
        //   OFF にすると従来どおり mode2 を mode1 と同じ扱いにする (A/B 比較用)。
        [Tooltip("PMX mode2(物理演算+ボーン位置合わせ)を再現する。OFFで従来どおりmode1と同一扱い")]
        public bool EnableBoneMergeMode = true;

        [Header("Correction (PmxEditorの補正層再現)")]
        // [物理+ボーン位置合わせ] 再現: 書き戻し時、位置を「親ボーン(補正済)位置+親回転×bindオフセット」の
        // 階層再構成に置換し、物理の移動分を捨てる (回転は物理のまま)。補正OFF/ON対照データで式を確定済み。
        // 既定 false=従来(物理位置をそのまま書き戻し)。ヘルパは PmxPhysicsBuilder.ComputeAlignedBonePoses (共通)。
        [Tooltip("MMDの[ボーン位置合わせ]再現: 位置=親チェーン再構成(移動分を捨てる)/回転=物理。スカート/髪の貫通表示対策。")]
        public bool AlignBonePositions = false;

        // [Jointロック内部演算] 再現の第一形: 親側ジョイントの相対eulerをリミット超過分だけ α で戻す。
        // 0=無効(回転そのまま) / 1=完全clamp。MMD(補正ON)の超過8-14°は完全clampでないことを示すため中間値。
        // AlignBonePositions とセットで使う (位置だけONは有害と実測済: 深貫入47,749)。掃引結果で既定を更新予定。
        [Tooltip("回転をジョイント角度リミットへ戻す割合 (AlignBonePositionsとセットで使用)。0=無効, 1=完全clamp。")]
        [Range(0f, 1f)] public float AlignRotClampAlpha = 0.5f;


        [Header("Timing Diagnosis (Animator遅れの実測)")]
        // 1フレーム内の実行順序と「物理が見たボーン」vs「表示されるボーン」の遅れを実測する。
        // ONにすると約120描画フレームをログして自動OFF。観点:
        //  - FixedUpdate時のボーン位置(=物理が見る) と LateUpdate時(=Animator適用後, 表示に近い) の差 dFL。
        //    dFL が動きに比例して大きい → 体コライダーは表示より1フレーム古い姿勢 = 速い動きで脚が刺さる機構。
        [Tooltip("ONで約120フレーム、実行順序/dt/ボーン遅れをConsoleへログして自動OFF")]
        public bool DiagnoseTiming = false;
        // ★既定はブランク (2026-08-10)。"右ひざ" のようなモデル依存の名前を既定に置くと、
        //   別モデルでは黙って解決できないまま計測が始まってしまう。使うときに明示指定する。
        [Tooltip("遅れ計測に使う速く動くボーン名 (例: 右ひざ)。空のまま診断ONにすると警告して中止する")]
        public string DiagnoseBone = "";

        private int _diagLeft = 0;
        private Transform _diagTr;
        private Vector3 _diagFixedPos, _diagUpdatePos;
        private int _diagFixedCount; private float _diagDt; private int _diagSteps;
        private readonly List<float> _diagDFL = new(); // |Fixed - Late|
        private readonly List<float> _diagDUL = new(); // |Update - Late|
        private static float Dist(Vector3 a, Vector3 b)
        { float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z; return (float)System.Math.Sqrt(dx * dx + dy * dy + dz * dz); }

        void Update()
        {
            if (DiagnoseTiming)
            {
                DiagnoseTiming = false; _diagLeft = 120; _diagDFL.Clear(); _diagDUL.Clear();
                _diagTr = null;
                if (_model != null && _boneTransforms != null)
                    for (int i = 0; i < _model.BoneNames.Count && i < _boneTransforms.Length; i++)
                        if (_model.BoneNames[i] == DiagnoseBone) { _diagTr = _boneTransforms[i]; break; }
                // ボーンが解決できないと、集計とカウントダウンを回す LateUpdate のブロックが
                // まるごと素通りする = 計測が終わらず自動OFFもされない。始める前に弾く。
                if (_diagTr == null)
                {
                    _diagLeft = 0;
                    Debug.LogWarning($"[TimingDiag] 中止: ボーン \"{DiagnoseBone}\" を解決できません" +
                        (string.IsNullOrEmpty(DiagnoseBone) ? " (DiagnoseBone が空です。例: 右ひざ)" : "") +
                        "。DiagnoseBone に、このモデルに実在する速く動くボーン名を入れてから再実行してください。");
                }
                else
                    Debug.Log($"[TimingDiag] 開始 bone={DiagnoseBone} fixedDeltaTime={Time.fixedDeltaTime:F4} FTS={FixedTimeStep:F4} SubSteps={SubSteps}");
            }
            if (_diagLeft > 0 && _diagTr != null) _diagUpdatePos = _diagTr.position;
        }

        [Header("Import Check (取り込み経路の検証)")]
        // ★このエンジンは「Unityのボーン座標 = PMXネイティブ座標」を前提に、境界の座標変換を
        //   単位スケールのみの恒等写像にしている(下の MmdToUnityPos 参照)。これが成立するのは
        //   GLB を **UniGLTF** で取り込んだ場合だけである:
        //     mmd2gltf が PMX→glTF で Z を反転し、UniGLTF が glTF→Unity で再度 Z を反転して相殺する。
        //   Unity 標準の glTF インポーター(glTFast)は代わりに X を反転するため相殺が起きず、
        //   スケルトンは PMX に対して Y軸180°回った状態で取り込まれる。剛体は extras.mmd の
        //   raw PMX 座標のまま構築されるので基準が食い違い、髪やスカートが体の反対側(正面)へ出る。
        // ★この食い違いは Unity も UniGLTF も一切エラーを出さない (2026-08-12 に実際に踏んだ)。
        //   だから起動時に自前で突き合わせる。検出できれば原因究明は数分で済む。
        [Tooltip("起動時にボーン配置とPMXバインド位置を突き合わせ、取り込み時の軸変換の食い違いを検出してLogErrorする")]
        public bool CheckImportConvention = true;

        [Header("Debug")]
        // ★既定OFF (2026-08-10)。自作エンジンが既定の物理になり通常のシーンで常駐するため、
        //   剛体100個超のギズモを常時描くのはSceneビューのノイズと描画コストにしかならない。
        //   剛体の位置ズレや取付を目で追いたいときだけONにする。
        [Tooltip("Sceneビューに剛体ギズモを描く。デバッグ時のみON推奨 (剛体は100個超あり重い)")]
        public bool DrawGizmos = false;

        private PmxPhysicsBuilder _builder;
        private PmxPhysicsModel _model;
        private Transform[] _boneTransforms;   // BoneIndex -> Transform
        private int _startupResetCountdown = 0; // >0 の間、LateUpdate で posed 姿勢へ再整合する

        void Start()
        {
            if (Source == InputSource.Glb) { if (!string.IsNullOrEmpty(GlbPath)) LoadGlb(GlbPath); }
            else { if (!string.IsNullOrEmpty(PmxPath)) LoadPmx(PmxPath); }
        }

        // PMX 直読み。
        public void LoadPmx(string path) => BuildAndInit(PmxReader.LoadFile(path));

        // GLB の extras.mmd 経由。UnitScale は extras.mmd の値を優先する
        // (GLB のメッシュ/スケルトンはその scale で import されているため、表示境界を一致させる必要がある)。
        public void LoadGlb(string path)
        {
            var model = GlbPhysicsReader.LoadFile(path, out float unitScale, out var warnings);
            if (warnings != null)
                foreach (var w in warnings) Debug.LogWarning($"[MmdPhysics][GLB] {w}");
            if (unitScale > 0f && System.Math.Abs(unitScale - UnitScale) > 1e-6f)
            {
                Debug.Log($"[MmdPhysics][GLB] UnitScale を extras.mmd の値 {unitScale} に設定 (Inspector {UnitScale} を上書き)");
                UnitScale = unitScale;
            }
            BuildAndInit(model);
        }

        // 入力経路に依らない共通の初期化 (物理駆動ロジックの共通化)。起動時に FK-rest リセットを必ず呼ぶ。
        private void BuildAndInit(PmxPhysicsModel model)
        {
            _model = model;
            _builder = PmxPhysicsBuilder.Build(_model);
            _builder.World.Gravity = new Vec3(0f, -Gravity, 0f);
            _builder.World.SolverIterations = SolverIterations;
            _builder.World.SubSteps = SubSteps;
            _builder.World.FixedTimeStep = FixedTimeStep;
            _builder.World.UseSplitImpulse = ContactSplitImpulse;
            _builder.World.UseJointSplitImpulse = JointSplitImpulse;
            _builder.World.EnableSleeping = EnableSleeping;
            ResolveBones();
            // 剛体を動かす前に検査する。この時点のスケルトンはバインド姿勢
            // (Animator がフレーム0を書くのは Start より後) なので PMX バインドと直接比較できる。
            if (CheckImportConvention) CheckImportConventionCore(false);
            ResetPhysicsToBones();
            // アニメがフレーム0を適用するのは Start より後(Update→LateUpdate 間)。この時点の
            // リセットはバインド基準なので、LateUpdate で posed 姿勢へ再整合し直す予約を入れる。
            _startupResetCountdown = PoseResetDelayFrames > 0 ? PoseResetDelayFrames : 0;
            CheckTimestepAlignment();
        }

        // 物理刻みと Unity の FixedUpdate 間隔が食い違うと、内部ステップが実時間で不均一になり
        // 髪/スカートがカクついて見える (ジャダー)。起動時に1度だけ整列 or 警告する。
        private void CheckTimestepAlignment()
        {
            if (AlignUnityFixedTimestep)
            {
                if (System.Math.Abs(Time.fixedDeltaTime - FixedTimeStep) > 1e-6f)
                {
                    Time.fixedDeltaTime = FixedTimeStep;
                    Debug.Log($"[MmdPhysics] Time.fixedDeltaTime を {FixedTimeStep:F6} に整列しました " +
                              $"(毎FixedUpdateでちょうど1ステップ=等間隔更新。SubSteps={SubSteps} で実効刻み {FixedTimeStep / SubSteps:F6})");
                }
                return;
            }
            float ratio = FixedTimeStep / Time.fixedDeltaTime;
            if (System.Math.Abs(ratio - Mathf_Round(ratio)) > 1e-3f)
                Debug.LogWarning($"[MmdPhysics] 物理刻みが Unity と整列していません: FixedTimeStep={FixedTimeStep:F5} / Time.fixedDeltaTime={Time.fixedDeltaTime:F5} " +
                    $"(比 {ratio:F3})。内部ステップが実時間で不均一(例 20ms/40ms交互)になり、髪やスカートがカクついて見えます。" +
                    "対策: AlignUnityFixedTimestep を ON にするか、FixedTimeStep=1/60・SubSteps=1 にして Fixed Timestep も 0.0166667 に合わせてください。");
        }
        private static float Mathf_Round(float v) => (float)System.Math.Round(v);

        /// <summary>物理開始/リセット時に、全剛体を現在のボーン姿勢へ整合させる
        /// (MMD の物理演算リセット相当)。フレーム0で脚が曲がっていても動的剛体がバインド位置に
        /// 取り残されて貫入するのを防ぐ。LoadPmx 後および任意のタイミングで呼べる。</summary>
        public void ResetPhysicsToBones()
        {
            if (_builder == null) return;
            // FK-rest で統一: 物理ボーン(スカート/髪)はスケルトンの姿勢を使わず親から前計算する。
            // スケルトンの物理ボーンに前フレームの物理結果が残っていても正しい開始状態になる。
            _builder.ResetBodiesToBonePoseFk(BoneWorldOrNull);
        }

        // ボーンindex → ワールド姿勢 (MMD座標)。Transform 未解決なら null (バインド維持)。
        private RigidTransform? BoneWorldOrNull(int boneIndex)
        {
            if (boneIndex < 0 || _boneTransforms == null ||
                boneIndex >= _boneTransforms.Length || _boneTransforms[boneIndex] == null)
                return null;
            var tr = _boneTransforms[boneIndex];
            return new RigidTransform(UnityToMmdRot(tr.rotation), UnityToMmdPos(tr.position));
        }

        private void ResolveBones()
        {
            _boneTransforms = new Transform[_model.BoneNames.Count];
            if (ModelRoot == null) return;
            var map = new Dictionary<string, Transform>();
            foreach (var t in ModelRoot.GetComponentsInChildren<Transform>())
                map[t.name] = t;
            for (int i = 0; i < _model.BoneNames.Count; i++)
                if (map.TryGetValue(_model.BoneNames[i], out var tr))
                    _boneTransforms[i] = tr;
        }

        /// <summary>取り込み経路の検証。Inspector で右クリック → "Check import convention"。
        /// Play 前(バインド姿勢)に実行するのが最も確実。</summary>
        [ContextMenu("Check import convention")]
        public void CheckImportConventionNow()
        {
            if (_model == null)
            {
                try
                {
                    if (Source == InputSource.Glb) { if (!string.IsNullOrEmpty(GlbPath)) _model = GlbPhysicsReader.LoadFile(GlbPath, out _, out _); }
                    else if (!string.IsNullOrEmpty(PmxPath)) _model = PmxReader.LoadFile(PmxPath);
                }
                catch (System.Exception e) { Debug.LogWarning($"[取り込み検査] モデル読込失敗: {e.Message}"); }
            }
            if (_model != null && ModelRoot != null) ResolveBones();
            CheckImportConventionCore(true);
        }

        // ボーンの配置を PMX バインド位置と突き合わせ、恒等以外の軸変換が挟まっていれば
        // 原因と対処法つきで LogError する。判定はモデルのシーン配置に影響されない:
        // ModelRoot のローカルへ落として並進・回転・スケールを除去し、さらに重心を引いてから比較する。
        private void CheckImportConventionCore(bool verbose)
        {
            if (ModelRoot == null || _model == null || _boneTransforms == null)
            {
                if (verbose) Debug.LogWarning("[取り込み検査] 実行不可 (ModelRoot / モデル / ボーンのいずれかが未設定)。");
                return;
            }
            int n = _model.BoneNames.Count;
            if (_boneTransforms.Length < n) n = _boneTransforms.Length;
            if (_model.BonePositions.Count < n) n = _model.BonePositions.Count;

            var ux = new float[n]; var uy = new float[n]; var uz = new float[n];
            var px = new float[n]; var py = new float[n]; var pz = new float[n];
            int m = 0, unresolved = 0;
            for (int i = 0; i < n; i++)
            {
                var tr = _boneTransforms[i];
                if (tr == null) { unresolved++; continue; }
                var l = ModelRoot.InverseTransformPoint(tr.position);
                ux[m] = l.x / UnitScale; uy[m] = l.y / UnitScale; uz[m] = l.z / UnitScale;
                var p = _model.BonePositions[i];
                px[m] = p.x; py[m] = p.y; pz[m] = p.z;
                m++;
            }
            if (m < 4)
            {
                Debug.LogError($"[取り込み検査] ボーンを {m}/{n} 本しか解決できません (未解決 {unresolved})。" +
                    "ModelRoot がモデルのルートを指しているか、取り込み時にボーン名が変えられていないかを確認してください " +
                    "(Unity 標準の glTF インポーターは重複するノード名にサフィックスを付けます)。");
                return;
            }

            float cux = 0, cuy = 0, cuz = 0, cpx = 0, cpy = 0, cpz = 0;
            for (int i = 0; i < m; i++) { cux += ux[i]; cuy += uy[i]; cuz += uz[i]; cpx += px[i]; cpy += py[i]; cpz += pz[i]; }
            cux /= m; cuy /= m; cuz /= m; cpx /= m; cpy /= m; cpz /= m;

            // 候補の軸変換それぞれで残差 RMS を測り、最も合うものを選ぶ。
            //   一致    = mmd2gltf と UniGLTF の ReverseZ が相殺した期待どおりの状態
            //   Y180    = 取り込み側が Z でなく X を反転した (Unity 標準の glTF インポーター等)
            //   Z/X鏡映 = 掌性が反転している (境界に余分な符号反転が残っている)
            string[] names = { "一致", "Y軸180度回転", "Z鏡映", "X鏡映" };
            int[] sx = { 1, -1, 1, -1 };
            int[] sz = { 1, -1, -1, 1 };
            int best = 0; float bestRms = float.MaxValue, identityRms = 0f;
            for (int c = 0; c < names.Length; c++)
            {
                double acc = 0;
                for (int i = 0; i < m; i++)
                {
                    float dx = (ux[i] - cux) - sx[c] * (px[i] - cpx);
                    float dy = (uy[i] - cuy) - (py[i] - cpy);
                    float dz = (uz[i] - cuz) - sz[c] * (pz[i] - cpz);
                    acc += dx * dx + dy * dy + dz * dz;
                }
                float rms = (float)System.Math.Sqrt(acc / m);
                if (c == 0) identityRms = rms;
                if (rms < bestRms) { bestRms = rms; best = c; }
            }
            // 許容量はモデルの広がりに対する相対値 (単位や体格に依存しないため)。
            double sacc = 0;
            for (int i = 0; i < m; i++)
            { float dx = px[i] - cpx, dy = py[i] - cpy, dz = pz[i] - cpz; sacc += dx * dx + dy * dy + dz * dz; }
            float spread = (float)System.Math.Sqrt(sacc / m);
            float tol = 0.02f * (spread > 1e-3f ? spread : 1f);

            string head = $"[取り込み検査] ボーン {m}/{n}" + (unresolved > 0 ? $" (未解決 {unresolved})" : "") +
                $" 残差RMS: 一致={identityRms:F4} / 最良={names[best]}={bestRms:F4} / 許容={tol:F4} (PMX単位)";

            if (best == 0)
            {
                if (identityRms <= tol)
                {
                    if (verbose) Debug.Log(head + " → OK。座標系は期待どおりです。");
                }
                else if (verbose)
                {
                    Debug.LogWarning(head + " → 軸変換は正しい(「一致」が最良)が残差が大きめです。" +
                        "Play 中に実行するとアニメで姿勢が変わっているため大きく出ます。Play 前に実行してください。" +
                        "Play 前でも大きい場合は UnitScale の不一致か、GLB とこのコンポーネントが参照するモデルが別物である可能性があります。");
                }
                return;
            }

            Debug.LogError(head + "\n" +
                $"→ ★スケルトンが PMX に対して「{names[best]}」の状態で取り込まれています。" +
                "剛体は GLB の extras.mmd にある raw PMX 座標のまま構築されるため基準が食い違い、" +
                "髪やスカートが体の反対側(正面)へ出ます。\n" +
                "原因はほぼ確実に GLB の取り込みに使ったインポーターです。.glb を選択して Inspector の見出しを見てください。\n" +
                "  UniGLTF なら『<名前> Import Settings (Glb Scripted Importer)』と表示されます。" +
                "『(Gltf Importer)』など他の名前なら、それが原因です。\n" +
                "対処: Package Manager から Unity の glTF インポーターを削除 → UniGLTF (com.vrmc.gltf) を導入 → " +
                ".glb を Reimport (効かない場合は .glb と .meta を消して入れ直す) → 物理の配線をやり直す。");
        }

        void FixedUpdate()
        {
            if (_builder == null) return;

            // --- TimingDiag: FixedUpdate = 物理が見るボーン姿勢 (このフレームのPush前)。 ---
            if (_diagLeft > 0 && _diagTr != null && _diagFixedCount == 0) _diagFixedPos = _diagTr.position;

            // 1. ボーン追従剛体に目標姿勢を渡す (物理前)。
            PushBonesToKinematic();


            // 2. 物理ステップ。
            _builder.World.StepSimulation(Time.fixedDeltaTime);
            if (_diagLeft > 0) { _diagFixedCount++; _diagDt = Time.fixedDeltaTime; _diagSteps += _builder.World.LastStepsRun; }

            // 3. ボーンへの書き戻しは LateUpdate で行う。
            //    ★2026-08-10: ここで書き戻すと Animator に上書きされる。
            //      Unityのフレームは FixedUpdate → Update → [Animator適用] → LateUpdate の順なので、
            //      FixedUpdate で書いた物理姿勢は、そのボーンにカーブがあると Animator に必ず潰される。
            //      症状: クリップがスカート/髪のカーブ(レストポーズの定数キーでも可)を持つモデルで、
            //      再生1フレーム目から揺れ物がレストポーズのまま固定される(コロン式で発現)。
            //      IA は本体のみベイクで揺れ物カーブが無いため表面化していなかった。
        }

        // 起動直後の数フレームだけ、アニメが確定させた「フレーム0姿勢」に対して物理を再整合する。
        // Animator は Update と LateUpdate の間で姿勢を書くため、ここでは posed 骨格が反映済み。
        // FK-rest リセットで動的剛体(スカート/髪)を posed 骨格の周りへ置き直し、バインド→フレーム0の
        // 瞬間移動で生じる脚への深い貫入(突き抜け)平衡を回避する。指定フレーム経過後はライブ物理へ。
        void LateUpdate()
        {

            // --- TimingDiag: LateUpdate = Animator(Normal)適用後。表示に最も近い姿勢。 ---
            if (_diagLeft > 0 && _diagTr != null)
            {
                var late = _diagTr.position;
                float dFL = _diagFixedCount > 0 ? Dist(_diagFixedPos, late) : -1f;
                float dUL = Dist(_diagUpdatePos, late);
                if (dFL >= 0) _diagDFL.Add(dFL);
                _diagDUL.Add(dUL);
                if (_diagLeft > 110 || dFL > 0.02f) // 最初の10フレームは全部、以降は大きい遅れのみ出力
                    Debug.Log($"[TimingDiag] F{Time.frameCount} fixedCalls={_diagFixedCount} steps={_diagSteps} dt={_diagDt:F4} | bone Fixed=({_diagFixedPos.x:F3},{_diagFixedPos.y:F3},{_diagFixedPos.z:F3}) Late=({late.x:F3},{late.y:F3},{late.z:F3}) dFL={dFL:F4} dUL={dUL:F4}");
                _diagFixedCount = 0; _diagSteps = 0;
                if (--_diagLeft == 0)
                {
                    _diagDFL.Sort(); _diagDUL.Sort();
                    float MedOf(List<float> v) => v.Count > 0 ? v[v.Count / 2] : 0f;
                    float MaxOf(List<float> v) => v.Count > 0 ? v[v.Count - 1] : 0f;
                    Debug.Log($"[TimingDiag] 要約: dFL(物理が見た姿勢と表示姿勢の差) 中央={MedOf(_diagDFL):F4} 最大={MaxOf(_diagDFL):F4} | dUL(Update vs Late=Animator書込タイミング) 中央={MedOf(_diagDUL):F4} 最大={MaxOf(_diagDUL):F4}"
                        + " 判定: dFL,dULとも大 → AnimatorはUpdate後に書く=物理は1フレーム古い体を見ている(遅れ確定)。dFL≈0 → 遅れなし=別因。");
                }
            }

            if (_builder == null) return;

            // アニメのループ境界/巻き戻し/シークで骨格が飛んだら、起動時の再整合を張り直す。
            // (判定は再整合そのものより前に置く。飛んだフレームの物理を捨てるのではなく、
            //  起動時と同じ「数フレームかけて整合させる」経路へ載せる。)
            if (_startupResetCountdown <= 0 && TeleportResetThreshold > 0f)
            {
                float th2 = TeleportResetThreshold * TeleportResetThreshold;
                int over = 0, total = 0;
                foreach (var link in _builder.BoneLinks)
                {
                    if (link.Mode != PhysicsMode.BoneFollow || link.BoneIndex < 0) continue;
                    var bw = BoneWorldOrNull(link.BoneIndex);
                    if (!bw.HasValue) continue;
                    total++;
                    var d = (bw.Value * link.BodyOffsetFromBone).Origin - link.Body.KinematicTarget.Origin;
                    if (d.LengthSquared > th2) over++;
                }
                int need = System.Math.Max(1, (int)System.Math.Ceiling(total * TeleportResetMinFraction));
                if (over >= need && over > 0)
                    _startupResetCountdown = PoseResetDelayFrames > 0 ? PoseResetDelayFrames : 1;
            }

            // 起動直後: アニメがフレーム0を適用した後の posed 骨格へ物理を再整合する。
            if (_startupResetCountdown > 0)
            {
                ResetPhysicsToBones();
                _startupResetCountdown--;
            }

            // ★物理 -> ボーンの書き戻しはここ (Animator の適用より後) で行う。
            //   FixedUpdate で書くと、揺れ物ボーンにカーブを持つクリップでは Animator に
            //   毎フレーム上書きされ、物理が一切見えなくなる (FixedUpdate 側のコメント参照)。
            //   LateUpdate なら Animator の後なので、揺れ物は必ず物理が勝つ。
            //   体のボーン(BoneFollow)には書き戻さないので、ダンスの動きはそのまま残る。
            PullPhysicsToBones();
        }

        private void PushBonesToKinematic()
        {
            // 駆動式は共通ヘルパに集約 (2026-08-09 hairfid誤配置事故の再発防止)。
            // 旧実装は未解決ボーンで Identity フォールバック=原点へテレポートし得た。ヘルパは null=前回維持で安全。
            _builder.ApplyKinematicTargets(BoneWorldOrNull);
        }

        private void PullPhysicsToBones()
        {
            // 補正層再現: 位置=親チェーン再構成 / 回転=物理 (共通ヘルパ)。
            //
            // ★PMX mode2 (DynamicBoneMerge = 物理演算+ボーン位置合わせ) の実装 (2026-08-10)。
            //   mode2 は「ボーンへの出力で、位置だけを親チェーン由来にし、回転は物理のまま」という
            //   *書き戻し側* の仕様であって、シミュレーションを拘束するものではない。
            //   ComputeAlignedBonePoses が元からその計算 (PmxEditorの補正層再現) をしており、
            //   従来は AlignBonePositions という全剛体一律のトグルにだけ繋がれていた。
            //   ここで mode2 の剛体に限り常時適用する。
            //   ※最初の実装で「剛体そのものを毎ステップ位置固定し並進速度をゼロにする」方式を試したが、
            //     旋回時に遠心力を担う並進速度まで消えて髪が軸へ collapse した (Tda式で振幅1.16→2.19、
            //     最小半径2.73→1.70)。シミュレーションには触れないのが正しい。
            bool needAligned = AlignBonePositions || (EnableBoneMergeMode && _builder.HasBoneMergeBodies);
            RigidTransform?[] aligned = needAligned ? _builder.ComputeAlignedBonePoses(BoneWorldOrNull, AlignRotClampAlpha, AlignBonePositions) : null;
            foreach (var link in _builder.BoneLinks)
            {
                if (link.Mode == PhysicsMode.BoneFollow) continue;
                if (link.BoneIndex < 0 || _boneTransforms == null ||
                    link.BoneIndex >= _boneTransforms.Length) continue;
                var tr = _boneTransforms[link.BoneIndex];
                if (tr == null) continue;

                // 補正を使うのは「全体トグルON」または「この剛体が mode2」のとき。
                bool useAligned = aligned != null && aligned[link.BoneIndex].HasValue &&
                                  (AlignBonePositions ||
                                   (EnableBoneMergeMode && link.Mode == PhysicsMode.DynamicBoneMerge));

                // body = bone * offset  ->  bone = body * offset^-1
                var boneWorld = useAligned
                    ? aligned[link.BoneIndex].Value
                    : link.Body.WorldTransform * link.BodyOffsetFromBone.Inverse();

                // ★NaN ガード (2026-08-10)。物理が発散して NaN を吐くと、そのまま Transform へ
                //   書き込まれてスケルトンが壊れ、毎フレーム大量のエラーが出て原因も埋もれる。
                //   ここで止めて剛体を静止させ、最初の1回だけ名指しで警告する。
                //   ※これは対症療法。発散そのものはエンジン側で潰すこと
                //   （既知の原因: 陽的ばねの過剰な力積 → Constraints.ApplySprings で安定化クランプ済み）。
                if (IsFinite(boneWorld.Origin) && IsFinite(boneWorld.Rotation))
                {
                    tr.position = MmdToUnityPos(boneWorld.Origin);
                    tr.rotation = MmdToUnityRot(boneWorld.Rotation);
                }
                else if (!_nanReported)
                {
                    _nanReported = true;
                    string bone = link.BoneIndex < _model.BoneNames.Count ? _model.BoneNames[link.BoneIndex] : $"#{link.BoneIndex}";
                    Debug.LogError($"[MmdPhysics] 物理が発散して NaN になりました (最初の検出: ボーン '{bone}')。" +
                        "このボーンへの書き戻しを停止し、以後の NaN は無視します。" +
                        "剛体のばね定数が大きすぎる/質量が小さすぎるモデルで起きます。" +
                        $"FixedTimeStep({FixedTimeStep:F5})を小さくするか SubSteps を増やすと改善することがあります。", this);
                }
            }
        }

        private static bool IsFinite(Vec3 v) =>
            !(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z) ||
              float.IsInfinity(v.x) || float.IsInfinity(v.y) || float.IsInfinity(v.z));

        private static bool IsFinite(Quat q) =>
            !(float.IsNaN(q.x) || float.IsNaN(q.y) || float.IsNaN(q.z) || float.IsNaN(q.w) ||
              float.IsInfinity(q.x) || float.IsInfinity(q.y) || float.IsInfinity(q.z) || float.IsInfinity(q.w));

        private bool _nanReported;

        private RigidTransform BoneWorld(int boneIndex)
        {
            if (boneIndex < 0 || _boneTransforms == null ||
                boneIndex >= _boneTransforms.Length || _boneTransforms[boneIndex] == null)
                return RigidTransform.Identity;
            var tr = _boneTransforms[boneIndex];
            return new RigidTransform(UnityToMmdRot(tr.rotation), UnityToMmdPos(tr.position));
        }

        // --- 座標変換 (MMD ネイティブ <-> Unity, 単位スケールのみ。Z反転なし) ---
        // メッシュ/スケルトンは mmd2gltf(ReverseZ) → UniGLTF(ReverseZ) の二重ReverseZが相殺し、
        // Unityボーンは既にPMXネイティブ座標値になっている。物理剛体もGlbPhysicsReaderがraw PMXで構築。
        // 従って境界は「単位スケールのみ」の真の等長変換にする(以前の3回目Z反転=鏡映バグを除去)。
        // 実測(DumpZ)でX-Z掌性の符号反転=鏡映を確認済み(PMX+1.583 vs U2M-1.583)。
        public Vector3 MmdToUnityPos(Vec3 v) => new(v.x * UnitScale, v.y * UnitScale, v.z * UnitScale);
        public Vec3 UnityToMmdPos(Vector3 v) => new(v.x / UnitScale, v.y / UnitScale, v.z / UnitScale);
        public static Quaternion MmdToUnityRot(Quat q) => new(q.x, q.y, q.z, q.w);
        public static Quat UnityToMmdRot(Quaternion q) => new(q.x, q.y, q.z, q.w);

        // Z符号確定用の診断: Play中に Inspector で本コンポーネントを右クリック→"Dump Z history"。
        // UnityToMmdPos(ボーン世界位置) を PMXバインド位置と成分比較する。
        //   X一致・Zだけ符号反転 → 鏡像(ブリッジの余分なZ反転) / X,Z両方反転 → メッシュ側Y180° / 全一致 → 座標無罪。
        [ContextMenu("Dump Z history")]
        public void DumpZHistory()
        {
            // PhysX既定時など Start が走っていない場合に備え、計測に必要な _model/_boneTransforms を自前構築。
            // 物理は駆動しない(Unityボーンの世界位置とPMXバインドを比較するだけ)。ModelRoot と GlbPath/PmxPath は必要。
            if (_model == null)
            {
                try
                {
                    if (Source == InputSource.Glb) { if (!string.IsNullOrEmpty(GlbPath)) _model = GlbPhysicsReader.LoadFile(GlbPath, out _, out _); }
                    else if (!string.IsNullOrEmpty(PmxPath)) _model = PmxReader.LoadFile(PmxPath);
                }
                catch (System.Exception e) { Debug.LogWarning($"[DumpZ] モデル読込失敗: {e.Message}"); }
            }
            // stale配列や物理未構築との不整合を避けるため、_model があれば毎回ボーンを再解決して長さを揃える。
            if (_model != null && ModelRoot != null) ResolveBones();
            if (_model == null || _boneTransforms == null)
            {
                Debug.LogWarning($"[DumpZ] 初期化不可 (model={( _model==null?"null":"ok")} bones={(_boneTransforms==null?"null":"ok")} ModelRoot={(ModelRoot==null?"未設定":"ok")} Source={Source} GlbPath='{GlbPath}' PmxPath='{PmxPath}')。ModelRootとパスを設定して実行してください。");
                return;
            }
            // 判定は体側(物理で書き戻されない)ボーンで行う。髪/モミアゲ等は物理書き戻しで鏡像と物理変位が混ざるため除外。
            // 頭(z=+0.193)と上半身2(z=-0.213)は符号が逆=両方が符号反転かつ絶対値一致なら鏡像の決定的証拠。
            // つま先系(|z|≈2.15)は大きい値で丸めに惑わされない確証用。
            string[] targets = {
                "頭", "上半身2", "上半身", "下半身", "右腕", "左腕",
                "右つま先", "左つま先", "右つま先ＩＫ", "左つま先ＩＫ"
            };
            var sb = new System.Text.StringBuilder("[DumpZ] 体側ボーン : UnityToMmd(bone.pos) vs PMXバインド\n");
            // モデルのシーン配置(並進/回転)を明示。これが非恒等だと world 基準の物理駆動に影響する。
            if (ModelRoot != null)
            {
                var e = ModelRoot.eulerAngles; var pos = ModelRoot.position; var sc = ModelRoot.lossyScale;
                sb.AppendLine($"  ModelRoot: pos=({pos.x:F3},{pos.y:F3},{pos.z:F3}) euler=({e.x:F1},{e.y:F1},{e.z:F1}) scale=({sc.x:F3},{sc.y:F3},{sc.z:F3})");
            }
            var U2M = new System.Collections.Generic.Dictionary<string, Vec3>();
            var PMX = new System.Collections.Generic.Dictionary<string, Vec3>();
            int nSafe = System.Math.Min(_model.BoneNames.Count, System.Math.Min(_boneTransforms.Length, _model.BonePositions.Count));
            for (int i = 0; i < nSafe; i++)
            {
                if (System.Array.IndexOf(targets, _model.BoneNames[i]) < 0) continue;
                var tr = _boneTransforms[i]; if (tr == null) continue;
                var m = UnityToMmdPos(tr.position); var p = _model.BonePositions[i];
                U2M[_model.BoneNames[i]] = m; PMX[_model.BoneNames[i]] = p;
                sb.AppendLine($"  {_model.BoneNames[i],-8}: U2M=({m.x,7:F3},{m.y,7:F3},{m.z,7:F3}) PMX=({p.x,7:F3},{p.y,7:F3},{p.z,7:F3})");
            }
            // ★配置(並進・回転)に依存しない鏡映判定: X-Z平面の掌性(符号付き面積)が反転すれば reflection=鏡像。
            //   横方向ベクトル(左腕-右腕) と 上下方向ベクトルの水平成分(頭-下半身) の2D外積の符号を PMX と U2M で比較。
            string verdict = "判定不能(基準ボーン不足: 右腕/左腕/頭/下半身/上半身2 が要る)";
            bool Has(string n) => U2M.ContainsKey(n);
            string vRef = Has("下半身") ? "下半身" : (Has("上半身") ? "上半身" : null);
            string vTop = Has("頭") ? "頭" : (Has("上半身2") ? "上半身2" : null);
            if (Has("右腕") && Has("左腕") && vRef != null && vTop != null)
            {
                Vec3 latP = PMX["左腕"] - PMX["右腕"], latU = U2M["左腕"] - U2M["右腕"];      // 横(主にX)
                Vec3 sagP = PMX[vTop] - PMX[vRef], sagU = U2M[vTop] - U2M[vRef];              // 縦(水平成分にZ)
                float crossP = latP.x * sagP.z - latP.z * sagP.x;   // X-Z平面の符号付き面積
                float crossU = latU.x * sagU.z - latU.z * sagU.x;
                float dyTop = System.Math.Abs((U2M[vTop].y - U2M[vRef].y) - (PMX[vTop].y - PMX[vRef].y));
                bool reflected = crossP * crossU < 0f;
                bool magOk = System.Math.Abs(System.Math.Abs(crossP) - System.Math.Abs(crossU)) <= 0.15f * System.Math.Max(System.Math.Abs(crossP), 1e-3f);
                verdict = $"掌性 PMX={crossP:F3} U2M={crossU:F3} (縦Y差={dyTop:F3}) → "
                    + (reflected ? (magOk ? "★鏡映(reflection)=鏡像確定→ブリッジZ反転除去でFix" : "★鏡映だが絶対値差大(回転/歪み混在,要確認)")
                                 : "掌性同符号=鏡映なし(座標無罪 or Y180回転)");
            }
            sb.AppendLine($"[判定] {verdict}");
            Debug.Log(sb.ToString());
        }

        void OnDrawGizmos()
        {
            if (!DrawGizmos || _builder == null) return;
            foreach (var body in _builder.Bodies)
            {
                Gizmos.color = body.Mode == PhysicsMode.BoneFollow
                    ? Color.cyan
                    : (body.Mode == PhysicsMode.Dynamic ? Color.green : Color.yellow);
                var p = MmdToUnityPos(body.WorldTransform.Origin);
                var rot = MmdToUnityRot(body.WorldTransform.Rotation);
                DrawShape(body.Shape, p, rot);
            }
        }

        private void DrawShape(CollisionShape shape, Vector3 pos, Quaternion rot)
        {
            var m = Gizmos.matrix;
            // 形状サイズは PMX ネイティブ単位なので UnitScale を掛けて位置と揃える。
            Gizmos.matrix = UnityEngine.Matrix4x4.TRS(pos, rot, Vector3.one * UnitScale);
            switch (shape)
            {
                case SphereShape s:
                    Gizmos.DrawWireSphere(Vector3.zero, s.Radius);
                    break;
                case BoxShape b:
                    Gizmos.DrawWireCube(Vector3.zero, new Vector3(b.HalfExtents.x, b.HalfExtents.y, b.HalfExtents.z) * 2f);
                    break;
                case CapsuleShape c:
                    Gizmos.DrawWireSphere(new Vector3(0, c.HalfHeight, 0), c.Radius);
                    Gizmos.DrawWireSphere(new Vector3(0, -c.HalfHeight, 0), c.Radius);
                    break;
            }
            Gizmos.matrix = m;
        }
    }
}
