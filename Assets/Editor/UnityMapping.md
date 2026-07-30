# Unity / lilToon への写し方

`extras.mmd` の**データの意味**は
[エクスポーター側の仕様書](https://github.com/masaka1024/mmd2gltf-gui/blob/v1.2.0/mmd2gltf/extrasMmd_schema.md)
が正本です。このファイルは、それを **Unity（URP + lilToon）でどう実装したか**を
記録します。仕様ではなく実装事項なので、エクスポーター側には持ち込みません。

---

## 描画順（renderQueue）

仕様側の要求は「材質番号順の合成を保つこと」です。Unity では以下の2帯構成で
実現しています（Tda式ミクV4X・IA で実証済み）。

| alphaClass | キュー | 理由 |
|---|---|---|
| `"mask"`（`origTexture` で半透明へ昇格したもの） | AlphaTest帯 `2452 ＋ 材質番号` | 深度を書きつつ、真の半透明より先に描画する。透け髪やレンズ越しの物体が正しく見える |
| `"blend"` | Transparent帯 `3000 ＋ 材質番号` | MMD の材質順合成を再現する |

`origTexture` があるものを一律に Transparent（3000）へ昇格させると、透明キュー内が
サブメッシュ番号順になるうえ TwoPass が深度を書くため、**前髪より後番号の
メガネ・レンズが前髪の深度に落ちて消えます**（Tda式ミクV4X で実測）。
共有テクスチャを使うモデルは全マテリアルが `origTexture` を持ちうるため、
昇格の条件を `alphaClass` で分けることが必須です。

## マテリアル

| extras.mmd | lilToon | 備考 |
|---|---|---|
| `sphereMode` = 1 | 乗算スフィア | PMX 標準の並び（1=乗算, 2=加算）。仕様書の記載と実装が逆だった時期があり、黒潰れの原因になった |
| `sphereMode` = 2 | 加算スフィア | |
| `sphereMode` = 3 | 未対応 | サブテクスチャ。MatCap の誤適用を防ぐため警告ログを出してスキップ |
| `toonTexture` | 個別トゥーン | glTF `textures[]` の番号だが Unity 側サブアセット順とは一致しないため、**名前で照合**する |
| `toonShared` | 共有トゥーン | MMD 標準の `toon01`〜`toon10.bmp` をプロジェクト内からファイル名検索して自動割り当て（利用者が用意する） |
| `flags` bit4 (16) | `_UseOutline` | 併せて `_OutlineColor` / `_OutlineWidth` を設定 |
| `edgeSize` | `_OutlineWidth` | 換算係数 0.08 は暫定値 |
| `origTexture` | ベースカラー差し替え | prebake 前の無加工テクスチャ。半透明 TwoPass と組み合わせて実アルファを復元する |

### 実装上の注意

- `lilToon.Editor` アセンブリは `autoreferenced: false` のため `using lilToon` が
  使えません。型名文字列によるリフレクションで
  `lilToonInspector.SetupMaterialWithRenderingMode` を呼びます。
- 同メソッドは `isoutl` 引数を明示しないとアウトライン込みの実体シェーダーに
  切り替わりません。3引数の簡易オーバーロードは ambient static に依存して
  不安定なため、**全引数を明示する7引数版**を使います。
- UniGLTF は標準マテリアルスロットから参照されない画像（トゥーン / スフィア）を
  テクスチャアセット化しません。GLB の bufferView を直接辿って生画像を抽出し、
  新規アセットとして保存する必要があります。
- `ambient` / `specular` / `specularPower` は未対応です。lilToon に MMD の
  specular に相当する色付きハイライトのプロパティがなく、近似の効果が
  軽微と判断して保留しています。

### 陰影オーバーレイの濃さ

眉・まつ毛まわりのオーバーレイは、本家よりやや薄いほうが好まれる場合があるため、
半透明画素（α&lt;250）のみを `tune_semiAlphaFactor` 倍に減衰させるスライダーを
用意しています（既定 0.7、肌の不透明画素は不変）。

---

## UniGLTF 側の既知の制約

- **モーフターゲットの sparse 形式を正しく読めません。** 口パクのモーフが
  別部位の変形として現れる症状が出ます。エクスポート時に
  `morph_mode="dense"` を指定してください（UniGLTF 前提のモデルは dense を標準に）。
