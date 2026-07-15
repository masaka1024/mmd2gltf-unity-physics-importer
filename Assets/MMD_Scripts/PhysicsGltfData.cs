using System;
using System.Collections.Generic;
using UnityEngine;

namespace Mmd2GltfImporter
{
    // ─────────────────────────────────────────────
    //  剛体データ（★実際のJSONキー名に一致させた版）
    //
    //   実物のキーは全部スネークケースだった：
    //     pos / rot / shape(int) / linear_damping / angular_damping
    //   shape は数値： 0=球, 1=箱, 2=カプセル
    // ─────────────────────────────────────────────
    [Serializable]
    public class RigidBodyData
    {
        public string name;
        public string name_en;
        public int bone;
        public int group;
        public int no_collision_mask;

        public int shape;          // 0=sphere, 1=box, 2=capsule
        public List<float> size;
        public List<float> pos;    // モデル原点基準の絶対座標
        public List<float> rot;    // オイラー角（ラジアン）

        public float mass;
        public float linear_damping;
        public float angular_damping;
        public float restitution;
        public float friction;
        public int mode;           // 0=Kinematic(ボーン追従), 1/2=物理
    }

    // ─────────────────────────────────────────────
    //  ジョイントデータ（実キー名対応・変更なし）
    // ─────────────────────────────────────────────
    [Serializable]
    public class JointData
    {
        public string name;
        public string name_en;
        public int type;

        public int rigid_a = -1; // 親側（connectedBody になる）
        public int rigid_b = -1; // 子側（ConfigurableJoint を付ける側）

        public List<float> pos;
        public List<float> rot;

        public List<float> pos_min;
        public List<float> pos_max;
        public List<float> rot_min;
        public List<float> rot_max;

        public List<float> spring_pos;
        public List<float> spring_rot;
    }
}
