# 配置について

- `Editor/` → Unityプロジェクトの `Assets/Editor/` へ（エディタ拡張のためEditorフォルダ必須）
- `MMD_Scripts/` → `Assets/MMD_Scripts/` へ（ランタイム＋開発ツール。Editorフォルダの外に置くこと）
- 旧 `MmdWaistSoftLimit.cs` を使用していた場合は削除してください（機能はインポーター本体へ統合済み）
- `.meta` ファイルを一緒に配置するとGUIDが固定され、アップデート時に参照が切れません
