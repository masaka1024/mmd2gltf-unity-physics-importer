> **【履歴】この文書は PhysX 版（〜2026-08-09）についての記録です。**
> 現行の物理は自作 Bullet 互換エンジンに置き換わっており、以下の数値・調整パラメータは
> **現行版には当てはまりません**。当時の計測と判断を残すためのアーカイブです。

# README 追記文面（コピペ用・日英）

---

## ライブ物理の再現度について（スカートの「翻り」）

本インポーターのライブ物理は、MMDの挙動を計測ベースで追い込んでいます。
現状の到達点は次の通りです：

- **平時の揺れ**（歩き・小さな動き）：MMDの物理ベイクと統計的にほぼ一致
- **急なターン時の翻り**：MMD比 **約8割** の振幅

完全一致に届かない理由は不具合ではなく、構造的なものです。MMDモデルの揺れ物設定は、
MMD の物理エンジンの拘束の解き方を前提に、作者が見栄えを目で追い込んだ「作品」です。
楽譜（PMXの数値）は完全に持ち運べても、別の楽器（別エンジン）で演奏すると
響きが変わる——本ツールの二層構造は、まさにこの状況のために設計されています：

| 用途 | 推奨経路 | 再現度 |
|---|---|---|
| 振付が決まっているコンテンツ（MV・鑑賞） | **フルキーベイク**（MMDで物理ベイク→全ボーン変換） | MMDの動きをそのまま再生 |
| インタラクティブ・未知のモーション | **ライブ物理**（本インポーター） | 平時ほぼ一致／翻り約8割・安定動作 |

ライブ物理の推奨設定は調整パネルの既定値に焼き込み済みです
（パネルの「リセット」で適用できます）。

**ロードマップ**：物理バックエンドを差し替え可能にする設計を予定しています。
v2でインターフェースを分離し、v3でMMDと同系統のソルバによるバックエンド追加を検討します。

---

## About Live-Physics Fidelity (Skirt "Flare")

The live physics of this importer is tuned against measured data from the original MMD behavior.
Current status:

- **Everyday sway** (walking, small motions): statistically matches the original physics bake
- **Skirt flare on fast turns**: about **80%** of the original amplitude

The remaining gap is structural, not a bug. Sway setups in MMD models are hand-tuned works of
art, refined by their authors against the constraint-solving behavior of the original engine.
The sheet music (PMX values) ports perfectly, but playing it on a different instrument
(a different physics engine) changes the sound. This is exactly what the two-layer design
of this toolchain is for:

| Use case | Recommended path | Fidelity |
|---|---|---|
| Fixed choreography (MV / showcase) | **Full-key bake** (bake physics in MMD, convert all bones) | Plays the original motion as-is |
| Interactive / unknown motion | **Live physics** (this importer) | Near-match at rest, ~80% flare, stable |

Recommended live-physics settings are baked into the tuning panel defaults
(press "Reset" in the panel to apply).

**Roadmap**: a pluggable physics-backend design is planned — interface separation in v2,
and a backend based on the same solver family as the original engine under consideration for v3.
