# `extras.mmd` スキーマ（参照）

**このファイルは仕様の正本ではありません。**
`extras.mmd` のフィールド定義・型・意味・ビットの解釈・座標系の約束は、
エクスポーター側リポジトリで一元管理しています。

## 正本

https://github.com/masaka1024/mmd2gltf-gui/blob/v1.2.0/mmd2gltf/extrasMmd_schema.md

物理（剛体・ジョイント）の変換済みビュー `physicsGltf` については、
同リポジトリの `mmd2gltf/physicsGltf_schema.md` を参照してください。

## このインポーターが実装している版

- **v1.2.0**

リンクは `main` ではなくタグを指しています。エクスポーター側で仕様が進んだとき、
このリンクは古い版を指したままになりますが、それは「まだ追従していない」という
事実がそのまま見える状態であり、意図した挙動です。追従したらタグを上げてください。

## Unity 固有の実装事項

キュー値・シェーダープロパティの対応など、Unity / lilToon への写し方は
仕様ではなく受け手側の実装事項です。→ [`UnityMapping.md`](UnityMapping.md)
