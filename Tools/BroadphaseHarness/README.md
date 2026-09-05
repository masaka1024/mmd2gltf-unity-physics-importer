# BroadphaseHarness — ブロードフェーズ早期棄却のビット不変検証 (Issue #2)

Unity なしで `Assets/MMD_Scripts/MmdPhysics/{Core,Pmx}` をそのままコンパイルし、
物理入り GLB を読み込んで `PhysicsWorld.UseBroadphasePrefilter` OFF / ON を同一の合成モーションで走らせ、
毎フレーム全剛体の姿勢・速度をビット単位で比較する。受理閾値に関わる A/B フラグの組み合わせ
(BulletContactThreshold / SpeculativeMargin / BulletShapeMargin / EnableSleeping など) でも繰り返す。

```bash
cd Tools/BroadphaseHarness
dotnet run -c Release -- "path/to/model_physics.glb" 1800
```

終了コード 0 = 全構成でビット一致。`UnityStub.cs` は MathTypes.cs の UnityEngine 型への明示変換を
通すための最小スタブ (Unity 本体では使わない。Assets の外にあるので Unity はコンパイルしない)。
