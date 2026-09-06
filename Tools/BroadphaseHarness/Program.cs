// Headless A/B harness: runs the MmdPhysics engine on a physics GLB with the broadphase
// prefilter OFF and ON under identical synthetic kinematic driving, and compares every
// body state (rotation, origin, linear/angular velocity) bit-for-bit after every frame.
// Repeats the comparison under several engine configurations that affect the broadphase /
// contact acceptance threshold.
using System;
using System.Collections.Generic;
using System.Diagnostics;
using BulletPhysics;
using BulletPhysics.Pmx;

static class Program
{
    sealed class Config
    {
        public string Name;
        public Action Static;                 // static flags (set before Build)
        public Action<PhysicsWorld> World;    // per-world settings
    }

    static int Main(string[] args)
    {
        string glb = args.Length > 0 ? args[0]
            : @"C:\Users\masa_\Downloads\mmd2gltf_full_20260726c\mmd2gltf_package3\csharp_test_out\IA_physics.glb";
        int frames = args.Length > 1 ? int.Parse(args[1]) : 1800;

        var model = GlbPhysicsReader.LoadFile(glb, out float unitScale, out var warnings);
        int nSphere = 0, nBox = 0, nCap = 0;
        foreach (var rb in model.RigidBodies) { if (rb.ShapeType == 0) nSphere++; else if (rb.ShapeType == 1) nBox++; else nCap++; }
        Console.WriteLine($"model '{model.ModelName}': bodies {model.RigidBodies.Count} (sphere {nSphere} / box {nBox} / capsule {nCap}) joints {model.Joints.Count} unitScale {unitScale} warnings {warnings.Count}");
        Console.WriteLine($"frames {frames} @60Hz");

        // JIT warm-up so the first measured run is not penalised.
        Run(model, true, 120, w => { }, 10, out _, quiet: true);
        Run(model, false, 120, w => { }, 10, out _, quiet: true);

        var configs = new List<Config>
        {
            new() { Name = "default (SubSteps=2, iters=10)", Static = Defaults, World = w => { w.SubSteps = 2; } },
            new() { Name = "SubSteps=1 (Quest setting)", Static = Defaults, World = w => { w.SubSteps = 1; } },
            new() { Name = "EnableSleeping=true", Static = Defaults, World = w => { w.SubSteps = 2; w.EnableSleeping = true; } },
            new() { Name = "BulletContactThreshold=false (fixed 0.02 band)", Static = () => { Defaults(); GjkEpa.BulletContactThreshold = false; }, World = w => { w.SubSteps = 2; } },
            new() { Name = "SpeculativeMargin=0.08 + BulletContactThreshold=false (AABB extra>0)", Static = () => { Defaults(); GjkEpa.BulletContactThreshold = false; GjkEpa.SpeculativeMargin = 0.08f; }, World = w => { w.SubSteps = 2; } },
            new() { Name = "SpeculativeMargin=0.08 + BulletContactThreshold=true", Static = () => { Defaults(); GjkEpa.SpeculativeMargin = 0.08f; }, World = w => { w.SubSteps = 2; } },
            new() { Name = "CollisionShape.BulletShapeMargin=true", Static = () => { Defaults(); CollisionShape.BulletShapeMargin = true; }, World = w => { w.SubSteps = 2; } },
            new() { Name = "SolveJointsFirst + UseSplitImpulse", Static = Defaults, World = w => { w.SubSteps = 2; w.SolveJointsFirst = true; w.UseSplitImpulse = true; } },
        };

        bool allOk = true;
        foreach (var c in configs)
        {
            Console.WriteLine();
            Console.WriteLine($"=== config: {c.Name} ===");
            c.Static();
            var off = Run(model, false, frames, c.World, 10, out double msOff);
            var on = Run(model, true, frames, c.World, 10, out double msOn);
            bool ok = Compare("prefilter OFF vs ON", off, on);
            Console.WriteLine($"  wall: OFF {msOff:F0} ms  ON {msOn:F0} ms  ({msOn / msOff * 100:F1}% of OFF)  -> {(ok ? "IDENTICAL" : "DIFFERENT")}");
            allOk &= ok;
        }
        Defaults();

        // Sanity: the comparison must be able to detect a real behavioural change.
        Console.WriteLine();
        Console.WriteLine("=== sanity: SolverIterations 10 vs 9 must DIFFER ===");
        var baseRun = Run(model, false, frames, w => { w.SubSteps = 2; }, 10, out _);
        var alt = Run(model, false, frames, w => { w.SubSteps = 2; }, 9, out _);
        bool sanity = !Compare("iters 10 vs 9", baseRun, alt);

        Console.WriteLine();
        Console.WriteLine(allOk ? "RESULT: IDENTICAL (bit-exact) in all configs" : "RESULT: DIFFERENT in at least one config");
        Console.WriteLine(sanity ? "sanity check: harness detects changes (OK)" : "sanity check FAILED: harness did not detect a real change");
        return allOk && sanity ? 0 : 1;
    }

    static void Defaults()
    {
        GjkEpa.BulletContactThreshold = true;
        GjkEpa.SpeculativeMargin = GjkEpa.SpeculativeMarginDefault;
        CollisionShape.BulletShapeMargin = false;
    }

    static List<float[]> Run(PmxPhysicsModel model, bool prefilter, int frames, Action<PhysicsWorld> cfg, int iters, out double ms, bool quiet = false)
    {
        var b = PmxPhysicsBuilder.Build(model);
        var w = b.World;
        w.FixedTimeStep = 1f / 60f;
        w.SolverIterations = iters;
        cfg(w);
        w.UseBroadphasePrefilter = prefilter;

        int n = w.Bodies.Count;
        var init = new RigidTransform[n];
        for (int i = 0; i < n; i++) init[i] = w.Bodies[i].WorldTransform;
        int kin = 0; for (int i = 0; i < n; i++) if (w.Bodies[i].IsKinematic) kin++;

        PhysicsWorld.ProfileEnabled = true;
        PhysicsWorld.ProfReset();
        var states = new List<float[]>(frames);
        var sw = Stopwatch.StartNew();
        for (int f = 0; f < frames; f++)
        {
            float t = (f + 1) / 60f;
            // Synthetic "dance": yaw + pitch about the model origin plus a large sway, so hair/skirt
            // bodies collide with the body colliders and with each other.
            var rot = Quat.FromAxisAngle(new Vec3(0, 1, 0), 0.9f * MathF.Sin(t * 2.1f))
                    * Quat.FromAxisAngle(new Vec3(1, 0, 0), 0.30f * MathF.Sin(t * 1.3f));
            var trans = new Vec3(3.0f * MathF.Sin(t * 1.7f), 1.0f * MathF.Abs(MathF.Sin(t * 3.1f)), 2.0f * MathF.Sin(t * 0.9f));
            for (int i = 0; i < n; i++)
            {
                var body = w.Bodies[i];
                if (!body.IsKinematic) continue;
                body.KinematicTarget = new RigidTransform(rot * init[i].Rotation, rot * init[i].Origin + trans);
            }
            w.StepSimulation(1f / 60f);

            var s = new float[n * 13];
            for (int i = 0; i < n; i++)
            {
                var body = w.Bodies[i]; int o = i * 13;
                s[o + 0] = body.WorldTransform.Rotation.x; s[o + 1] = body.WorldTransform.Rotation.y;
                s[o + 2] = body.WorldTransform.Rotation.z; s[o + 3] = body.WorldTransform.Rotation.w;
                s[o + 4] = body.WorldTransform.Origin.x; s[o + 5] = body.WorldTransform.Origin.y; s[o + 6] = body.WorldTransform.Origin.z;
                s[o + 7] = body.LinearVelocity.x; s[o + 8] = body.LinearVelocity.y; s[o + 9] = body.LinearVelocity.z;
                s[o + 10] = body.AngularVelocity.x; s[o + 11] = body.AngularVelocity.y; s[o + 12] = body.AngularVelocity.z;
            }
            states.Add(s);
        }
        ms = sw.Elapsed.TotalMilliseconds;
        PhysicsWorld.ProfileEnabled = false;
        if (quiet) return states;

        double sub = PhysicsWorld.ProfSubSteps;
        Console.WriteLine($"[prefilter={(prefilter ? "ON " : "OFF")} iters={iters} SubSteps={w.SubSteps}] bodies {n} (kinematic {kin}) pairs {w.DebugCollisionPairCount}  substeps {sub}");
        Console.WriteLine($"  broad {PhysicsWorld.ProfBroad:F1} ms  build {PhysicsWorld.ProfBuild:F1} ms  prepare {PhysicsWorld.ProfPrepare:F1} ms  solveJoint {PhysicsWorld.ProfSolveJoint:F1} ms  solveContact {PhysicsWorld.ProfSolveContact:F1} ms  total {ms:F0} ms");
        Console.WriteLine($"  per substep: pairs {PhysicsWorld.ProfPairs / sub:F0}  sphereRejected {PhysicsWorld.ProfPairsSphereRejected / sub:F0}  aabbRejected {PhysicsWorld.ProfPairsAabbRejected / sub:F0}  detectCalls {PhysicsWorld.ProfDetectCalls / sub:F1}  manifolds {PhysicsWorld.ProfManifolds / sub:F1}  contacts {PhysicsWorld.ProfContacts / sub:F1}");
        return states;
    }

    static bool Compare(string label, List<float[]> a, List<float[]> b)
    {
        int firstFrame = -1, firstIdx = -1; long diffCount = 0; float maxAbs = 0;
        for (int f = 0; f < a.Count; f++)
        {
            var x = a[f]; var y = b[f];
            for (int j = 0; j < x.Length; j++)
            {
                if (BitConverter.SingleToInt32Bits(x[j]) == BitConverter.SingleToInt32Bits(y[j])) continue;
                diffCount++;
                if (firstFrame < 0) { firstFrame = f; firstIdx = j; }
                float d = MathF.Abs(x[j] - y[j]); if (d > maxAbs) maxAbs = d;
            }
        }
        if (diffCount == 0) { Console.WriteLine($"  compare {label}: IDENTICAL over {a.Count} frames"); return true; }
        Console.WriteLine($"  compare {label}: {diffCount} differing values, first at frame {firstFrame} body {firstIdx / 13} comp {firstIdx % 13}, max |diff| {maxAbs:G6}");
        return false;
    }
}
