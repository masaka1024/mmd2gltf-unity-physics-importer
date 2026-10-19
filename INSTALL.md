# 配置について

`Assets/` の中身をそのまま Unity プロジェクトの `Assets/` へコピーしてください。

| コピー元 | コピー先 | 備考 |
|---|---|---|
| `Assets/Editor/` | `Assets/Editor/` | エディタ拡張。**Editor フォルダ必須** |
| `Assets/MMD_Scripts/` | `Assets/MMD_Scripts/` | ランタイム。**Editor フォルダの外に置くこと**（中に入れると実行時 missing 参照になる） |

`Assets/MMD_Scripts/MmdPhysics/` は物理エンジン本体です（同梱）。
正本は https://github.com/masaka1024/mmd2gltf-cs-physics で、**両リポジトリへ同じ修正を入れる**運用です。

`.meta` ファイルを一緒に配置すると GUID が固定され、アップデート時にシーン/プレハブの参照が切れません。

## `.glb` の取り込みは UniGLTF を推奨

推奨は UniGLTF です。Unity 標準の glTF インポーター（glTFast）でも、【1】物理を配線 のときに取り込み経路を
判定して補正するので物理は正しく動きます（モデル全体は Y 軸 180° 回って見えます。マテリアル変換は未検証）。
`.glb` を別のインポーターで取り込み直したときは物理の配線をやり直してください。詳細は
[README](README.md#glb-の取り込みは-unigltf-を推奨他のインポーターでも物理は動きます) にあります。

## 旧バージョン（PhysX 版）から更新する場合

2026-08-10 に PhysX 経路を撤去しました。以下は**手動で取り除いてください**。

- シーン上の `Rigidbody` / `ConfigurableJoint`（インポーターが生成していたもの）
- コライダー用の子オブジェクト（`col_*` / `rb_*` / `MMD_PhysicsRig`）
- 削除された補助スクリプト:
  `MmdPhysicsImportIndex` `MmdCollisionMask` `MmdCollisionGroupApplier` `MmdGravity`
  `MmdPhysicsWarmup` `MmdPhysicsWatcher` `MmdContactProbe` `MmdBulletMimicry`
  `MmdWaistSoftLimit` `MmdMotionStats` `MmdJointProbe` `MmdSpinTest`
  `MmdBakedPlaybackMode` `PhysicsGltfData`

放置すると `Missing (Mono Script)` と孤児コンポーネントがシーンに残ります。
