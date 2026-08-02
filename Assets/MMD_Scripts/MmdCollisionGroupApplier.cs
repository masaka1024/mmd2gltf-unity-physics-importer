using System.Collections.Generic;
using UnityEngine;

namespace Mmd2GltfImporter
{
    // ★PMXの衝突グループ(group 0〜15)と非衝突グループフラグ(no_collision_mask)を
    //   Unityで再現するためのランタイムコンポーネント。
    //
    //   Physics.IgnoreCollision はシーンやプレハブに永続化されないため、
    //   インポート時に計算した「衝突を無効化すべきコライダーペア」をここに保存し、
    //   再生開始時(Awake)に毎回適用し直す仕組みにしている。
    //   ※Editorフォルダの外に置くこと(中に置くと実行時missing参照になる)。
    public class MmdCollisionGroupApplier : MonoBehaviour
    {
        [System.Serializable]
        public struct IgnorePair
        {
            public Collider a;
            public Collider b;
        }

        [Tooltip("PMXのgroup/no_collision_maskから算出した、衝突させないコライダーペアの一覧")]
        public List<IgnorePair> ignorePairs = new List<IgnorePair>();

        private void Awake()
        {
            Apply();
        }

        // 保存済みペアへ IgnoreCollision を一括適用する。
        // 再生開始時に自動で呼ばれるほか、デバッグ用に手動でも呼べる。
        public void Apply()
        {
            int applied = 0;
            foreach (var p in ignorePairs)
            {
                if (p.a == null || p.b == null) continue;
                Physics.IgnoreCollision(p.a, p.b, true);
                applied++;
            }
            Debug.Log($"[MMD Physics] PMX衝突グループ: {applied} ペアの衝突を無効化しました。");
        }
    }
}
