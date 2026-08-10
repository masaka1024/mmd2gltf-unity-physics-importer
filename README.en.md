# mmd2gltf-unity-physics-importer

**[日本語](README.md) | English**

A Unity Editor extension that reads the MMD-specific data preserved in the `extras` region of a glTF (`.glb`) exported by `mmd2gltf-gui` — rigid-body physics settings and toon/sphere material settings — and converts them into something that actually runs in Unity.

> **Note**: This tool depends on the specific GLB layout produced by `mmd2gltf-gui` (custom data preserved under `extras.mmd`). It will not work with output from generic glTF exporters.

---

## Features

### Physics

Reads PMX rigid bodies and joints and drives them with a **custom Bullet-compatible physics engine** (`Assets/MMD_Scripts/MmdPhysics/`, bundled). One button wires everything up and secondary animation (hair, skirts, ribbons) comes alive.
Stock MMD behaviour is the target, but this is **not on par with it** (see "Known limitations").

- Reproduces PMX rigid bodies, joints, and physics modes (bone-follow / dynamic / dynamic + bone position merge)
- Pure C#; no native dependency (no BulletSharp etc.)
- Unity's PhysX is not used (see "Migration from PhysX" below)

### Materials

- Automatic conversion to [lilToon](https://github.com/lilxyzw/lilToon) (URP)
- Restores MMD toon textures, sphere maps (add/multiply), and outlines
- Extracts toon/sphere textures directly from the GLB binary (UniGLTF does not import them)
- Supports MMD's shared toons (`toon01`–`toon10`) using images you supply
- Results are saved as standalone `.mat` assets that survive re-import

### In-game hitboxes

Generates **detection-only colliders** from the PMX rigid-body definitions. These are separate from the physics simulation and are meant for hit detection in shooters/action games.

- Selectable scope (body parts only / hair & skirt only / all)
- Creates `isTrigger` colliders and **no Rigidbody**. They are simply parented to bones, so they add no physics cost and follow whether the bone is driven by the Animator or by physics
- Identify what was hit via `MmdHitbox.PartName` (`頭` / `上半身2` / `右ひざ` …)

```csharp
void OnTriggerEnter(Collider other) {
    var hit = other.GetComponent<MmdHitbox>();
    if (hit == null) return;
    damage *= hit.PartName == "頭" ? 2f : 1f;   // per-part damage
}
```

### Diagnostics

- **Clip inspector** — detects AnimationClip curves targeting secondary-animation bones and can produce a stripped copy
- **Skin binding inspector** — cross-checks SkinnedMeshRenderer bone references against the physics-driven bones
- Raw JSON dumps from the GLB (joints / materials / texture mapping / bone rotations)

### UI

- Japanese / English toggle

---

## Requirements

- Unity with the Universal Render Pipeline (targeting Unity 6)
- [UniGLTF](https://github.com/vrm-c/UniVRM) for glTF import
- [lilToon](https://github.com/lilxyzw/lilToon) for material conversion (not needed if you only want physics)
- A `.glb` produced by `mmd2gltf-gui`

---

## Setup

**Copy the contents of `Assets/` into your project's `Assets/`.**

```
Assets/
  Editor/                     Editor extensions (importer window, diagnostics)
  MMD_Scripts/
    MmdHitbox.cs              Hitbox marker (runtime)
    FreeCameraController.cs   Free camera for inspection
    MmdPhysics/               Physics engine (Core / Pmx / Unity)
```

> Everything under `Editor/` is editor-only. Everything under `MMD_Scripts/` is needed at runtime, so it must stay **outside** any `Editor` folder (otherwise you get missing references at play time).

To restore shared toons, place `toon01.bmp`–`toon10.bmp` (shipped with MMD) anywhere under `Assets` — they are located by filename.

---

## Usage

1. Import the `.glb` and place the model in the scene
2. Open **`MMD Physics > インポーター`** from the menu
3. Assign the scene model to the target field
4. **[1] Wire / Re-wire Physics** — this alone makes physics work
5. **[2] Convert Materials to lilToon** — independent of physics, run any time
6. **[3] Build Hitboxes** — only if you need them

Fine-tuning of physics (timestep, penetration handling) lives on the `MmdPhysicsBehaviour` component added by step 1.

---

## Migration from PhysX (2026-08-10)

Earlier versions drove secondary animation with Unity `Rigidbody` + `ConfigurableJoint` (PhysX) and exposed 49 tuning sliders (springs, damping, soft limits, per-part dials).

The custom Bullet-compatible engine now produces **noticeably closer-to-MMD behaviour than the PhysX
path did in practice**, so **the PhysX path has been removed** to avoid maintaining two implementations.

> This does not mean it matches stock MMD. Numerical fidelity has only ever been validated against a
> single model (IA), those numbers predate recent fixes, and some aspects — such as jitter at rest —
> are still **worse** than stock MMD (see "Known limitations").

| | Old | Current |
|---|---|---|
| Physics | Unity PhysX | Custom Bullet-compatible engine |
| Steps | bodies → joints → materials | wire physics → materials |
| Tuning sliders | 49 | 0 (physics settings moved to `MmdPhysicsBehaviour`) |
| Importer window | 3,541 lines | 1,160 lines |

**When updating from an older version**, manually remove any leftover `Rigidbody` / `ConfigurableJoint` components and the deleted helper scripts (`MmdGravity`, `MmdPhysicsWarmup`, `MmdCollisionMask`, …) from your scenes. Otherwise you will be left with `Missing (Mono Script)` entries and orphaned components.

---

## About the physics engine

The bundled `Assets/MMD_Scripts/MmdPhysics/` is a copy of an engine developed in its own repository.

**Source of truth: https://github.com/masaka1024/mmd2gltf-cs-physics**

> ⚠ **Fixes must be applied to both repositories.** After changing the engine, mirror the change into
> `Assets/MMD_Scripts/MmdPhysics/` here (the two are kept file-for-file identical).
> Design notes, measurements, and "things we tried that failed" live in the engine repo's `docs/`.

Layout differs slightly: the engine repo uses `Assets/MmdPhysics/{Core,Pmx,Unity,DevTools}`, while here it is
`Assets/MMD_Scripts/MmdPhysics/{Core,Pmx,Unity}` (the two `DevTools` files sit in `Unity/`; contents are identical).

---

## Known limitations

- **Jitter at rest**: secondary animation vibrates slightly more than stock MMD even when nearly static. This is an open issue in the engine (position correction feeds energy back into real velocity). See "静止時のジッタ" in the engine repo README.
- **Skirt physics**: approximated with simple sphere/box/capsule colliders, so collision and interpenetration are less natural than for hair.
- **`ambient` / `specular`**: lilToon has no equivalent of MMD's coloured highlight, so these are unsupported.
- **Shared toons**: the images are not included here (they ship with MMD). You must supply them.
- **UniGLTF morph import**: models exported with `morph_mode="sparse"` may show broken morphs — UniGLTF does not read sparse morph targets correctly. Use `morph_mode="dense"`.

---

## Design background

The tool started by creating dedicated rigid-body objects and writing their poses back to bones every frame, then was rebuilt around attaching `Rigidbody` directly to bones (inspired by [mmd-for-unity](https://github.com/ousttrue/mmd-for-unity)) on PhysX. It now replaces the physics itself with the custom engine.

Physics results are written back to bones in **`LateUpdate`**. Writing in `FixedUpdate` lets the Animator overwrite them every frame whenever the clip has curves for secondary-animation bones — even constant rest-pose keys are enough to hide the physics completely.

---

## Credits

- Physics design originally referenced [mmd-for-unity](https://github.com/ousttrue/mmd-for-unity)
- Material conversion uses [lilToon](https://github.com/lilxyzw/lilToon)
- Input `.glb` files are expected from `mmd2gltf-gui` (PMX/VMD → glTF 2.0 converter)

## License

MIT License / Copyright (c) 2026 masaka1024 — see [LICENSE](LICENSE) for the full text.
