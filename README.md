# mmd2gltf-unity-physics-importer

`mmd2gltf-gui` が出力した glTF（`.glb`）を Unity に取り込んだ後、モデルの `extras` 領域に保存されている MMD 固有データ（物理演算設定・トゥーン/スフィアマテリアル設定）を読み取り、Unity 上で実際に動く形（Rigidbody・ConfigurableJoint・lilToon マテリアル）へ自動変換する Editor 拡張です。

> **注意**：このツールは `mmd2gltf-gui` が出力する特定の GLB 形式（`extras.mmd` にスネークケース/キャメルケースで温存された独自データ）に依存しています。一般的な glTF エクスポーターの出力では動作しません。

---

## できること

### 物理演算
- MMD の剛体・ジョイント設定を読み取り、Unity の `Rigidbody` / `ConfigurableJoint` を自動生成
- 実際にメッシュを動かしているボーン Transform に直接 Rigidbody を付与する方式（[mmd-for-unity](https://github.com/ousttrue/mmd-for-unity) を参考にした設計）を採用しており、揺れ物の物理挙動がそのままメッシュへ反映される
- 髪・スカートを専用の衝突レイヤーに分離し、絡まりによる暴走や体へのめり込みを緩和
- 各種パラメータ（震え対策のダンピング下限、関節の遊び、コライダー縮小率など）を GUI 上のスライダーでその場調整可能

### マテリアル
- [lilToon](https://github.com/lilxyzw/lilToon) への自動変換（URP 対応）
- MMD のトゥーンテクスチャ・スフィアマップ（加算/乗算）・輪郭線（アウトライン）を復元
- UniGLTF が取り込まないトゥーン/スフィア用テクスチャを GLB バイナリから直接抽出
- MMD 標準の共有トゥーン（`toon01`〜`toon10`）にも対応（別途ユーザー側で用意した画像を使用）
- 変換結果は独立した `.mat` アセットとして保存され、プロジェクトの再インポートでも消えない

### UI
- 日本語 / English の切り替えボタン
- デバッグ用の確認ツール一式（JSON の生データ確認、ボーン回転の一括出力など）を折りたたみ表示

---

## 必要なもの

- Unity（Universal Render Pipeline を使用しているプロジェクト）
- [UniGLTF](https://github.com/vrm-c/UniVRM)（glTF のインポートに使用）
- [lilToon](https://github.com/lilxyzw/lilToon)（マテリアル変換に使用。物理演算のみを使う場合は不要）
- `mmd2gltf-gui` で変換した `.glb` ファイル

---

## セットアップ

以下の3ファイルをプロジェクトに配置してください。

| ファイル | 配置場所 |
|---|---|
| `MmdPhysicsImporterWindow.cs` | `Assets/Editor/` 配下 |
| `PhysicsGltfData.cs` | `Assets/Editor/` の外（通常フォルダ） |
| `MmdPhysicsImportIndex.cs` | `Assets/Editor/` の外（通常フォルダ） |

共有トゥーンを復元したい場合は、MMD 本体に同梱されている `toon01.bmp`〜`toon10.bmp`（計10枚）を、`Assets` 内の任意の場所に配置してください（ファイル名で自動検索されるため、配置場所は問いません）。

---

## 使い方

1. Unity メニューから `MMD > Physics Importer` を開く
2. Scene 上の対象モデルを「対象モデル」欄にセット
3. 必要に応じて「調整パネル」でパラメータを調整
4. **ボタン1**（剛体とコライダーを配置）→ **ボタン2**（ジョイントを結合）の順に実行
5. **ボタン3**（マテリアルを lilToon へ変換）を実行（物理設定とは独立して、いつでも実行可能）

---

## 既知の制限事項

- **スカートの物理**：単純な球/箱/カプセルのコライダーによる近似のため、髪と比べると衝突・めり込みの挙動が完全に自然ではありません。将来的にはクロスシミュレーション等への置き換えを検討中です。
- **`ambient` / `specular` マテリアル設定**：lilToon には MMD の色付きハイライトに相当するプロパティが無いため、現状は未対応です。
- **共有トゥーン**：画像そのものはこのリポジトリに含まれていません（MMD 本体の配布物のため）。利用者側で別途用意する必要があります。
- **UniGLTF のモーフインポート**：`mmd2gltf-gui` 側で `morph_mode="sparse"`（差分圧縮）を使ってエクスポートしたモデルで、モーフの見た目が崩れる場合があります。UniGLTF が sparse 形式のモーフターゲットを正しく読み込めないことが原因と特定済みです。エクスポート時は `morph_mode="dense"` を推奨します。

---

## 設計上の背景

このツールは当初「揺れる剛体を専用オブジェクトとして作り、毎フレームその姿勢をボーンへ書き戻す」方式で実装されていましたが、フィードバックループや自己参照ループによる不具合が頻発したため、[mmd-for-unity](https://github.com/ousttrue/mmd-for-unity) の設計（Rigidbody をボーン自体に直接付与する方式）を参考に全面的に作り直しています。この設計により、専用の書き戻し処理が不要になり、揺れ物の挙動が大幅に安定しました。

---

## クレジット

- 物理演算の設計は [mmd-for-unity](https://github.com/ousttrue/mmd-for-unity) を参考にしています
- マテリアル変換には [lilToon](https://github.com/lilxyzw/lilToon) を使用します
- 入力となる `.glb` は自作ツール `mmd2gltf-gui`（PMX/VMD → glTF 2.0 変換ツール）が生成したものを想定しています

## ライセンス

MIT License

Copyright (c) 2026 masaka1024

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
