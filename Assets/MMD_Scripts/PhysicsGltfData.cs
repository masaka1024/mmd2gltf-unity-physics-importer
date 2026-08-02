using System;
using System.Collections.Generic;
using UnityEngine;

namespace Mmd2GltfImporter
{
    // ─────────────────────────────────────────────
    //  剛体データ
    //
    //  extras.mmd には2系統の剛体・ジョイントが併存する（physicsGltf_schema.md）。
    //
    //   ① raw          … "rigidBodies" / "joints"（スネークケース、PMX生値・未スケール、
    //                     回転はオイラー角ラジアン、位置はモデル原点基準）
    //   ② physicsGltf  … 変換済み。距離は glTF シーン単位（unitScale 適用済み）、
    //                     回転はクォータニオン、位置は bone のローカル空間。
    //
    //  ②が使える場合はスケール推定・座標変換・オイラー順序の解釈がすべて不要に
    //  なるため、そちらを優先する。1つのクラスで両方のキー名を宣言しておき
    //  （JsonUtility は存在しないキーを既定値のまま残す）、読み込み後に
    //  AdoptPhysicsGltfFields() で②の値を共通スロットへ移す。
    //
    //  shape は数値： 0=球, 1=箱, 2=カプセル
    // ─────────────────────────────────────────────
    [Serializable]
    public class RigidBodyData
    {
        // ── 両系統で共通のキー ──
        public string name;
        public string name_en;
        public int bone;
        public int group;

        public int shape;          // 0=sphere, 1=box, 2=capsule
        public List<float> size;   // 球:[r] / 箱:[hx,hy,hz]半寸法 / カプセル:[r, 高さ]

        public float mass;
        public float restitution;
        public float friction;
        public int mode;           // 0=Kinematic(ボーン追従), 1=物理, 2=物理+ボーン

        // ── raw 版のキー ──
        public List<float> pos;    // モデル原点基準の絶対座標（PMX生値・未スケール）
        public List<float> rot;    // オイラー角（ラジアン）
        public float linear_damping;
        public float angular_damping;
        public int no_collision_mask;

        // ── physicsGltf 版のキー ──
        public string space;            // "boneLocal" | "world"
        public List<float> position;    // space基準（スケール適用済み）
        public List<float> rotation;    // space基準の quat (x,y,z,w)
        public float linearDamping;
        public float angularDamping;

        // 【重要】立っているビットは「そのグループと衝突する」を意味する
        // （Bullet の collision filter mask と同じ。PMXエディタのチェック表示＝
        //  非衝突とは反転している）。衝突条件は双方向で、
        //  (1<<A.group & B.mask) && (1<<B.group & A.mask) のときだけ衝突する。
        // キー名は PMX のフィールド名「非衝突グループフラグ」由来で据え置き。
        public int noCollisionMask;

        public bool IsBoneLocal
        {
            get { return string.IsNullOrEmpty(space) || space == "boneLocal"; }
        }

        /// <summary>physicsGltf 由来の値を共通スロットへ移す。position/rotation は
        /// 基準空間の意味が raw と違うため移さず、呼び出し側で直接使う。</summary>
        public void AdoptPhysicsGltfFields()
        {
            linear_damping = linearDamping;
            angular_damping = angularDamping;
            no_collision_mask = noCollisionMask;
            rot = null; // 回転は rotation(quat) を使う
        }
    }

    // ─────────────────────────────────────────────
    //  ジョイントデータ（すべて type: 0 = 6DOFスプリング拘束）
    // ─────────────────────────────────────────────
    [Serializable]
    public class JointData
    {
        public string name;
        public string name_en;
        public int type;

        // ── raw 版のキー ──
        public int rigid_a = -1; // 親側（connectedBody になる）
        public int rigid_b = -1; // 子側（ConfigurableJoint を付ける側）

        public List<float> pos;
        public List<float> rot;

        public List<float> pos_min;   // PMX生値・未スケール
        public List<float> pos_max;
        public List<float> rot_min;   // ラジアン（PMX軸のまま）
        public List<float> rot_max;

        public List<float> spring_pos;
        public List<float> spring_rot;

        // ── physicsGltf 版のキー ──
        public int rigidA = -1;
        public int rigidB = -1;
        public int refBone = -1;
        public string space;

        public List<float> position;
        public List<float> rotation;          // quat (x,y,z,w)

        public List<float> linearLimitMin;    // glTF単位
        public List<float> linearLimitMax;
        public List<float> angularLimitMin;   // ラジアン。glTF軸への鏡映済み
        public List<float> angularLimitMax;

        public List<float> springPosition;
        public List<float> springRotation;

        /// <summary>physicsGltf 由来の値を共通スロットへ移す。移動制限が glTF 単位に、
        /// 角度制限が glTF 軸（X/Y反転・min/max入替済み）になるのが raw との違い。</summary>
        public void AdoptPhysicsGltfFields()
        {
            rigid_a = rigidA;
            rigid_b = rigidB;
            pos_min = linearLimitMin;
            pos_max = linearLimitMax;
            rot_min = angularLimitMin;
            rot_max = angularLimitMax;
            spring_pos = springPosition;
            spring_rot = springRotation;
        }
    }
}