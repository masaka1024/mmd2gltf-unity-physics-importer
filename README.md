# mmd2gltf-unity-physics-importer

**日本語 | [English](README.en.md)**

`mmd2gltf-gui` が出力した glTF（`.glb`）を Unity に取り込んだ後、モデルの `extras` 領域に保存されている MMD 固有データ（物理演算設定・トゥーン/スフィアマテリアル設定）を読み取り、Unity 上で実際に動く形へ自動変換する Editor 拡張です。

> **注意**：このツールは `mmd2gltf-gui` が出力する特定の GLB 形式（`extras.mmd` にスネークケース/キャメルケースで温存された独自データ）に依存しています。一般的な glTF エクスポーターの出力では動作しません。

---

## できること

### 物理演算

MMD の剛体・ジョイント設定を読み取り、**自作の Bullet 互換物理エンジン**（`Assets/MMD_Scripts/MmdPhysics/`、同梱）で駆動します。ボタン1つで配線が終わり、揺れ物（髪・スカート・リボン等）が動きます。
本家 MMD の挙動を目標に作っていますが、**同等ではありません**（「既知の制限事項」参照）。

- PMX の剛体・ジョイント・物理モード（ボーン追従 / 物理演算 / 物理+ボーン位置合わせ）を再現
- 外部ネイティブライブラリ（BulletSharp 等）に依存しない純 C# 実装
- Unity の PhysX は使いません（下記「PhysX からの移行」参照）

### マテリアル

- [lilToon](https://github.com/lilxyzw/lilToon) への自動変換（URP 対応）
- MMD のトゥーンテクスチャ・スフィアマップ（加算/乗算）・輪郭線（アウトライン）を復元
- UniGLTF が取り込まないトゥーン/スフィア用テクスチャを GLB バイナリから直接抽出
- MMD 標準の共有トゥーン（`toon01`〜`toon10`）にも対応（別途ユーザー側で用意した画像を使用）
- 変換結果は独立した `.mat` アセットとして保存され、プロジェクトの再インポートでも消えない

### ゲーム内の当たり判定（ヒットボックス）

PMX の剛体定義から **判定専用のコライダー**を生成します。物理演算とは別物で、シューティングやアクションでの被弾判定に使えます。

- 生成範囲を選べる（体パーツのみ / 髪・スカートのみ / すべて）
- `isTrigger` のコライダーのみを作り、**Rigidbody は付けません**。ボーンの子に置くだけなので物理計算は一切増えず、アニメでも物理でもボーンに追従します
- 当たった部位は `MmdHitbox.PartName`（`頭` / `上半身2` / `右ひざ` など）で判別できます

```csharp
void OnTriggerEnter(Collider other) {
    var hit = other.GetComponent<MmdHitbox>();
    if (hit == null) return;
    damage *= hit.PartName == "頭" ? 2f : 1f;   // 部位別ダメージ
}
```

### 診断ツール

- **クリップ診断** — AnimationClip が揺れ物ボーンのカーブを持っていないか検査し、除去した複製を作る
- **スキン結線検査** — SkinnedMeshRenderer の参照ボーンと揺れ物ボーンの結線を突き合わせる
- GLB の生 JSON ダンプ（ジョイント / 材質 / テクスチャ対応 / ボーン回転）

### UI

- 日本語 / English の切り替えボタン

---

## 必要なもの

- Unity（Universal Render Pipeline を使用しているプロジェクト。想定は Unity 6）
- [UniGLTF](https://github.com/vrm-c/UniVRM)（glTF のインポートに使用）
- [lilToon](https://github.com/lilxyzw/lilToon)（マテリアル変換に使用。物理演算のみを使う場合は不要）
- `mmd2gltf-gui` で変換した `.glb` ファイル

---

## セットアップ

**`Assets/` フォルダの中身をそのままプロジェクトの `Assets/` へコピー**してください。

```
Assets/
  Editor/                     エディタ拡張（インポーター本体・診断ツール）
  MMD_Scripts/
    MmdHitbox.cs              当たり判定のマーカー（ランタイム）
    FreeCameraController.cs   確認用フリーカメラ
    MmdPhysics/               物理エンジン本体（Core / Pmx / Unity）
```

> `Editor/` 配下はエディタ専用です。`MMD_Scripts/` 配下は実行時に必要なので **`Editor` フォルダの外**に置いてください（中に入れると実行時に missing 参照になります）。

共有トゥーンを復元したい場合は、MMD 本体に同梱されている `toon01.bmp`〜`toon10.bmp`（計10枚）を `Assets` 内の任意の場所に配置してください（ファイル名で自動検索されるため、場所は問いません）。

---

## 使い方

1. `.glb` を Unity に import し、Scene に配置する
2. メニューから **`MMD Physics > インポーター`** を開く
3. Scene 上の対象モデルを「対象モデル」欄にセット
4. **【1】物理を配線 / 再配線** — これだけで物理が動きます
5. **【2】マテリアルを lilToon へ変換** — 物理とは独立していつでも実行可
6. **【3】当たり判定を生成**（必要な場合のみ）

物理の細かい設定（刻み・貫入対策など）は、配線後にモデルへ付く `MmdPhysicsBehaviour` の Inspector にあります。

---

## PhysX からの移行（2026-08-10）

以前のバージョンは Unity の `Rigidbody` + `ConfigurableJoint`（PhysX）で揺れ物を動かし、
GUI に 49 個の調整スライダー（ばね・減衰・ソフトリミット・部位別ダイヤル等）がありました。

自作の Bullet 互換エンジンが**実用上 PhysX 版より本家に近い挙動**を出せるようになったため、
二重メンテを避けて **PhysX 経路は撤去**しました。

> 「本家と同等になった」という意味ではありません。数値での忠実度検証は IA 1 モデルでしか
> 行っておらず、その数値も直近の修正より前のものです。静止時の微振動など**本家より劣る点も
> 残っています**（下記「既知の制限事項」）。

| | 旧 | 現行 |
|---|---|---|
| 物理 | Unity PhysX | 自作 Bullet 互換エンジン |
| 手順 | 剛体配置 → ジョイント結合 → 材質 | 物理を配線 → 材質 |
| 調整スライダー | 49 個 | 0 個（物理側の設定は `MmdPhysicsBehaviour` へ） |
| インポーター本体 | 3,541 行 | 1,160 行 |

**旧版から更新する場合**、シーンに残っている `Rigidbody` / `ConfigurableJoint` と、削除された補助スクリプト（`MmdGravity` / `MmdPhysicsWarmup` / `MmdCollisionMask` 等）は手動で取り除いてください。放置すると `Missing (Mono Script)` と孤児コンポーネントが残ります。

---

## 物理エンジンについて

同梱の `Assets/MMD_Scripts/MmdPhysics/` は、独立したリポジトリで開発している自作エンジンの複製です。

**正本: https://github.com/masaka1024/mmd2gltf-cs-physics**

> ⚠ **同じ修正を両リポジトリへ入れる必要があります。** エンジン側を直したら、こちらの
> `Assets/MMD_Scripts/MmdPhysics/` へも反映してください（ファイル単位で完全一致させる運用）。
> 設計判断・実測データ・「試して失敗した記録」はエンジン側リポジトリの `docs/` にあります。

エンジン側の配置は `Assets/MmdPhysics/{Core,Pmx,Unity,DevTools}`、こちらは
`Assets/MMD_Scripts/MmdPhysics/{Core,Pmx,Unity}` です（`DevTools` の 2 ファイルは `Unity/` に置いています。内容は同一）。

---

## 既知の制限事項

- **静止時の微振動**：ほとんど静止した状態でも揺れ物が本家より細かく震えます。エンジン側の未解決課題です（位置補正が実速度へエネルギーを注入し続ける構造の問題）。詳細はエンジン側 README の「静止時のジッタ」節。
- **スカートの物理**：単純な球/箱/カプセルのコライダーによる近似のため、髪と比べると衝突・めり込みの挙動が完全に自然ではありません。
- **`ambient` / `specular` マテリアル設定**：lilToon には MMD の色付きハイライトに相当するプロパティが無いため未対応です。
- **共有トゥーン**：画像そのものはこのリポジトリに含まれていません（MMD 本体の配布物のため）。利用者側で別途用意する必要があります。
- **UniGLTF のモーフインポート**：`mmd2gltf-gui` 側で `morph_mode="sparse"`（差分圧縮）を使ってエクスポートしたモデルで、モーフの見た目が崩れる場合があります。UniGLTF が sparse 形式のモーフターゲットを正しく読み込めないことが原因と特定済みです。エクスポート時は `morph_mode="dense"` を推奨します。

---

## 設計上の背景

当初は「揺れる剛体を専用オブジェクトとして作り、毎フレームその姿勢をボーンへ書き戻す」方式で、次に [mmd-for-unity](https://github.com/ousttrue/mmd-for-unity) を参考にした「Rigidbody をボーン自体に直接付与する」PhysX 方式へ作り直しました。現行は物理そのものを自作エンジンに置き換えています。

物理結果のボーンへの書き戻しは **`LateUpdate`** で行います。`FixedUpdate` で書くと、揺れ物ボーンにカーブを持つ AnimationClip では Animator に毎フレーム上書きされ、物理が一切見えなくなるためです（レストポーズの定数キーだけでも起きます）。

---

## クレジット

- 物理演算の設計は当初 [mmd-for-unity](https://github.com/ousttrue/mmd-for-unity) を参考にしています
- マテリアル変換には [lilToon](https://github.com/lilxyzw/lilToon) を使用します
- 入力となる `.glb` は自作ツール `mmd2gltf-gui`（PMX/VMD → glTF 2.0 変換ツール）が生成したものを想定しています

## ライセンス

MIT License / Copyright (c) 2026 masaka1024 — 全文は [LICENSE](LICENSE) を参照してください。
