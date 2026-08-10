// ===========================================================================
// 物理バックエンドのトグル並立: PhysX版(既存 mmd2gltf インポーターが付ける
// Rigidbody + ConfigurableJoint)と、自作エンジン版(MmdPhysicsBehaviour)を排他切替する。
//
// 設計方針:
//   - Unity 組込み型 (Rigidbody / ConfigurableJoint) だけを操作するため、
//     既存インポーターのソースは一切変更しない (importer への依存も持たない)。
//   - Custom: 自作エンジンを有効化し、PhysX 剛体を「パーク」(isKinematic=true,
//     detectCollisions=false) して同じボーンを奪い合わないようにする。ConfigurableJoint も無効化。
//   - PhysX: 自作エンジンを無効化し、PhysX 剛体/Joint を元の状態へ復帰する。
//   - どちらも同時に同じボーン Transform を動かさない (排他)。
//   - 物理以外 (lilToon マテリアル / スキンバインディング / ベイク再生) には触れない。
//
// ※ Unity の PhysX 型 (Rigidbody/ConfigurableJoint) を使うため、UnityEngine 最小シムの
//   検証ハーネスからは除外し (#if)、Unity 実機でのみコンパイルされる。
// ===========================================================================
#if UNITY_2019_1_OR_NEWER
using System.Collections.Generic;
using UnityEngine;

namespace BulletPhysics.Unity
{
    [DisallowMultipleComponent]
    public sealed class MmdPhysicsBackendSwitch : MonoBehaviour
    {
        public enum Backend { Custom, PhysX }

        [Tooltip("Custom=自作エンジン / PhysX=既存インポーターの Rigidbody+ConfigurableJoint")]
        public Backend Mode = Backend.Custom;

        [Tooltip("自作エンジン (未指定ならこの GameObject 以下から自動検出)")]
        public MmdPhysicsBehaviour customEngine;

        [Tooltip("Custom時、PhysX側の診断/補助スクリプトも自動停止する (Custom運用では純粋な無駄。PhysXへ戻すと復帰)")]
        public bool DisablePhysXHelpers = true;

        // 停止対象 (型名で照合するので importer への参照/依存は持たない。名前を足せば対象を増やせる)。
        //  MmdJointProbe   : 72Jointを毎FixedUpdate計測+毎秒巨大文字列をDebug.Log (エディタでは特に重い)
        //  MmdMotionStats  : 101剛体を毎ステップ記録
        //  MmdGravity      : パーク済み108剛体へ毎FixedUpdate AddForce (完全に無駄)
        //  MmdCollisionMask: 保留ペアを毎FixedUpdate走査
        // ※MmdPhysicsWarmup は isKinematic を戻す張本人だが EnforceExclusive で無害化済み。
        //   止めたい場合はここに "MmdPhysicsWarmup" を足す。
        [Tooltip("停止するPhysX側スクリプトの型名 (importerへの依存を避けるため名前で照合)")]
        public string[] PhysXHelperTypeNames = { "MmdJointProbe", "MmdMotionStats", "MmdGravity", "MmdCollisionMask" };

        private readonly List<Behaviour> _disabledHelpers = new();

        private Rigidbody[] _rbs;
        private bool[] _origKinematic;
        private bool[] _origDetectCollisions;
        private bool _snapped;

        void Awake() { Snapshot(); ApplyBackend(); }

        void OnValidate() { if (Application.isPlaying && _snapped) ApplyBackend(); }

        // PhysX コンポーネントの初期状態を控える (元の isKinematic を PhysX 復帰で使う)。
        private void Snapshot()
        {
            _rbs = GetComponentsInChildren<Rigidbody>(true);
            _origKinematic = new bool[_rbs.Length];
            _origDetectCollisions = new bool[_rbs.Length];
            for (int i = 0; i < _rbs.Length; i++)
            {
                _origKinematic[i] = _rbs[i].isKinematic;
                _origDetectCollisions[i] = _rbs[i].detectCollisions;
            }
            if (customEngine == null) customEngine = GetComponentInChildren<MmdPhysicsBehaviour>(true);
            _snapped = true;
        }

        [ContextMenu("Apply Backend (現在のModeを適用)")]
        public void ApplyBackend()
        {
            if (!_snapped) Snapshot();
            bool custom = Mode == Backend.Custom;

            // 自作エンジン: Custom で有効、PhysX で無効。
            if (customEngine != null)
            {
                customEngine.enabled = custom;
                // 切替直後にボーンが PhysX 側で動いていた場合に備え、Custom へ入る時は
                // 現在のボーン姿勢へ FK-rest 再整合する (未ロードなら内部で無視される)。
                if (custom) customEngine.ResetPhysicsToBones();
            }

            // PhysX 剛体: Custom ではパーク (kinematic/衝突無効)、PhysX では元状態へ復帰。
            for (int i = 0; i < _rbs.Length; i++)
            {
                var rb = _rbs[i];
                if (rb == null) continue;
                if (custom) { rb.isKinematic = true; rb.detectCollisions = false; }
                else { rb.isKinematic = _origKinematic[i]; rb.detectCollisions = _origDetectCollisions[i]; }
            }
            // PhysX側の診断/補助スクリプト: Custom で停止、PhysX で復帰 (型名照合=importer非依存)。
            if (custom && DisablePhysXHelpers) DisableHelpers();
            else RestoreHelpers();

            // ConfigurableJoint は Behaviour ではないため .enabled で無効化できない。
            // だが Custom では上のループで全 Rigidbody を isKinematic=true にパークしており、
            // kinematic なボディは Joint の拘束/ドライブで動かされない=Joint は自動的に無効(inert)になる。
            // PhysX へ戻すと isKinematic が元に戻り Joint も自然に復帰するため、個別操作は不要。
        }

        // ★2026-08-09 実機バグ修正: パークは Awake の1回では保たない。
        // 既存インポーターの MmdPhysicsWarmup が FixedUpdate で isKinematic=false に戻す
        // (ログ「[MMD 始動] 101個の揺れ物を物理へ引き渡しました(t=0.22s)」) ため、
        // Custom 中でも PhysX がスカート/髪を動かし、自作エンジンと同じボーンを奪い合っていた
        // (Unityだけスカートが貫通し、ヘッドレス/CSVプレイヤーでは出なかった真因)。
        // 対策: Custom の間は毎 FixedUpdate でパークを再主張する (importer への依存は持たない)。
        void FixedUpdate()
        {
            if (!EnforceExclusive || Mode != Backend.Custom || _rbs == null) return;
            for (int i = 0; i < _rbs.Length; i++)
            {
                var rb = _rbs[i];
                if (rb == null) continue;
                if (!rb.isKinematic) { rb.isKinematic = true; _reparked++; }
                if (rb.detectCollisions) rb.detectCollisions = false;
            }
            if (_reparked > 0 && !_reparkLogged)
            {
                _reparkLogged = true;
                Debug.Log($"[BackendSwitch] Custom中にPhysX剛体が起こされたため再パークしました ({_reparked}件)。" +
                          "原因: 既存インポーターの Warmup/他スクリプトが実行時に isKinematic=false へ戻すため。" +
                          "以後も毎FixedUpdateで排他を維持します (EnforceExclusive)。");
            }
        }

        [Tooltip("Custom中、PhysX剛体が他スクリプトに起こされても毎フレーム再パークして排他を維持する (実機の貫通対策)")]
        public bool EnforceExclusive = true;
        private int _reparked; private bool _reparkLogged;

        private void DisableHelpers()
        {
            if (PhysXHelperTypeNames == null || PhysXHelperTypeNames.Length == 0) return;
            var all = GetComponentsInChildren<Behaviour>(true);
            var stopped = new List<string>();
            foreach (var bh in all)
            {
                if (bh == null || !bh.enabled) continue;
                string tn = bh.GetType().Name;
                bool hit = false;
                for (int i = 0; i < PhysXHelperTypeNames.Length; i++)
                    if (tn == PhysXHelperTypeNames[i]) { hit = true; break; }
                if (!hit) continue;
                bh.enabled = false;
                _disabledHelpers.Add(bh);
                stopped.Add(tn);
            }
            if (stopped.Count > 0)
                Debug.Log($"[BackendSwitch] Custom のためPhysX側スクリプトを停止しました: {string.Join(", ", stopped)} (計{stopped.Count}件)。PhysXへ戻すと復帰します。");
        }

        private void RestoreHelpers()
        {
            if (_disabledHelpers.Count == 0) return;
            int n = 0;
            foreach (var bh in _disabledHelpers) if (bh != null) { bh.enabled = true; n++; }
            _disabledHelpers.Clear();
            if (n > 0) Debug.Log($"[BackendSwitch] PhysX側スクリプトを復帰しました ({n}件)。");
        }

        [ContextMenu("Use Custom (自作エンジン)")]
        public void UseCustom() { Mode = Backend.Custom; ApplyBackend(); }

        [ContextMenu("Use PhysX (既存インポーター)")]
        public void UsePhysX() { Mode = Backend.PhysX; ApplyBackend(); }
    }
}
#endif
