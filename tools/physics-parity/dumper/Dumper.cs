// Physics parity dumper (C# side). See ../README.md.
//
// Runs a fixed scenario with the C# engine and writes every body's state after every frame as
// raw float32, in the same format as ../runner.ts (the TypeScript port).
//
//   Dumper <model.pmx> <out.bin> [scenario.bin] [perturbBody]
//     scenario.bin : written here (the TypeScript runner reads it, so both sides drive the bones
//                    with exactly the same float32 values)
//     perturbBody  : body index whose start position x is moved by 1 ulp (chaos control), -1 = none
//   env CFG = 0 (default settings) / 1 / 2 (two sets of non-default switches, to cover other paths)
//
// Dump format: header int32[4] = { 0x504D4450 ('PDMP'), bodies, frames, 13 },
// then per frame, per body: position(3) rotation(4) linear velocity(3) angular velocity(3).
using System;
using System.Diagnostics;
using System.IO;
using BulletPhysics;
using BulletPhysics.Pmx;

static class Dumper
{
    const int Frames = 420;
    const int Rec = 13;
    const int Magic = 0x504D4450;

    static int Main(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("usage: Dumper <model.pmx> <out.bin> [scenario.bin] [perturbBody]");
            return 2;
        }
        string pmx = args[0], outPath = args[1];
        string scenPath = args.Length > 2 ? args[2] : null;
        int perturb = args.Length > 3 ? int.Parse(args[3]) : -1;

        string cfg = Environment.GetEnvironmentVariable("CFG") ?? "0";
        if (cfg == "2") { CollisionShape.BulletShapeMargin = true; CapsuleShape.InertiaMargin = 0f; }

        var model = PmxReader.LoadFile(pmx);
        PmxPhysicsBuilder.MaxDynamicMass = 0;
        var b = PmxPhysicsBuilder.Build(model);
        var w = b.World;
        // Same settings as the Unity importer (MmdPhysicsBehaviour) and FacetGL
        w.Gravity = new Vec3(0f, -98f, 0f);
        w.SolverIterations = 10; w.SubSteps = 2; w.FixedTimeStep = 1f / 60f;
        w.UseSplitImpulse = false; w.UseJointSplitImpulse = false; w.EnableSleeping = false; w.UseBroadphasePrefilter = true;
        if (cfg == "1")
        {
            w.UseSplitImpulse = true; w.UseJointSplitImpulse = true; Joint.LinearLeverMode = 2; PersistentManifold.BulletManifoldPoints = false;
            Joint.SpringAsMotorRow = false; w.ContactRhsBullet = false; w.FrictionVelocityAligned = false; w.FrictionCombineMultiply = false;
            PhysicsWorld.BulletRotationIntegration = false; w.EnableSleeping = true; w.UseJointWarmStart = true; w.UseJointWarmStartAngular = true; w.SolveJointsFirst = true;
        }
        if (cfg == "2")
        {
            Joint.BulletLimitRowGating = false; Joint.AngularMixedAxes = false; Joint.BulletAngleConvention = false; Joint.LinearLeverMode = 0;
            w.ContactPoolOrder = true; GjkEpa.BulletContactThreshold = false; w.JointVelocityIterations = 15; PersistentManifold.SymmetricBreakingDistance = false;
        }

        var bind = model.BonePositions;
        b.ResetBodiesToBonePoseFk(i => new RigidTransform(Quat.Identity, bind[i]));
        if (perturb >= 0)
        {
            var body = w.Bodies[perturb];
            var t = body.WorldTransform;
            t.Origin.x = BitConverter.Int32BitsToSingle(BitConverter.SingleToInt32Bits(t.Origin.x) + 1);
            body.WorldTransform = t;
        }

        // Scenario per frame (qx, qy, qz, qw, ox, oy, oz): the whole skeleton is turned about the
        // root and moved. 0-59 rest, 60-239 shaking, 240-419 rest.
        var scen = new float[Frames * 7];
        for (int k = 0; k < Frames; k++)
        {
            Quat q = Quat.Identity; Vec3 o = Vec3.Zero;
            if (k >= 60 && k < 240)
            {
                double t = (k - 60) / 60.0;
                o = new Vec3((float)(3 * Math.Sin(2 * Math.PI * t)), (float)(0.5 * Math.Sin(4 * Math.PI * t)), (float)(1.5 * Math.Sin(2 * Math.PI * t * 0.5)));
                float ang = (float)(0.6 * Math.Sin(2 * Math.PI * t * 0.5));
                q = new Quat(0f, (float)Math.Sin(ang * 0.5), 0f, (float)Math.Cos(ang * 0.5));
            }
            scen[k * 7 + 0] = q.x; scen[k * 7 + 1] = q.y; scen[k * 7 + 2] = q.z; scen[k * 7 + 3] = q.w;
            scen[k * 7 + 4] = o.x; scen[k * 7 + 5] = o.y; scen[k * 7 + 6] = o.z;
        }
        if (!string.IsNullOrEmpty(scenPath))
        {
            var sb = new byte[scen.Length * 4];
            Buffer.BlockCopy(scen, 0, sb, 0, sb.Length);
            File.WriteAllBytes(scenPath, sb);
        }

        Vec3 root = bind[0];
        int nBodies = w.Bodies.Count;
        var dump = new float[Frames * nBodies * Rec];
        var sw = new Stopwatch();
        double steadyMs = 0; int steadyN = 0;
        for (int k = 0; k < Frames; k++)
        {
            var q = new Quat(scen[k * 7], scen[k * 7 + 1], scen[k * 7 + 2], scen[k * 7 + 3]);
            var o = new Vec3(scen[k * 7 + 4], scen[k * 7 + 5], scen[k * 7 + 6]);
            RigidTransform? Bw(int i) => new RigidTransform(q, q * (bind[i] - root) + root + o);
            sw.Restart();
            b.ApplyKinematicTargets(Bw);
            w.StepSimulation(1f / 60f);
            sw.Stop();
            if (k >= 20) { steadyMs += sw.Elapsed.TotalMilliseconds; steadyN++; }
            for (int i = 0; i < nBodies; i++)
            {
                var bd = w.Bodies[i]; int p = (k * nBodies + i) * Rec;
                var tr = bd.WorldTransform;
                dump[p] = tr.Origin.x; dump[p + 1] = tr.Origin.y; dump[p + 2] = tr.Origin.z;
                dump[p + 3] = tr.Rotation.x; dump[p + 4] = tr.Rotation.y; dump[p + 5] = tr.Rotation.z; dump[p + 6] = tr.Rotation.w;
                dump[p + 7] = bd.LinearVelocity.x; dump[p + 8] = bd.LinearVelocity.y; dump[p + 9] = bd.LinearVelocity.z;
                dump[p + 10] = bd.AngularVelocity.x; dump[p + 11] = bd.AngularVelocity.y; dump[p + 12] = bd.AngularVelocity.z;
            }
        }

        using (var fs = File.Create(outPath))
        using (var bw = new BinaryWriter(fs))
        {
            bw.Write(Magic); bw.Write(nBodies); bw.Write(Frames); bw.Write(Rec);
            var bytes = new byte[dump.Length * 4];
            Buffer.BlockCopy(dump, 0, bytes, 0, bytes.Length);
            bw.Write(bytes);
        }
        Console.WriteLine($"bodies={nBodies} joints={w.Joints.Count} frames={Frames} ms/frame={steadyMs / steadyN:F2}");
        return 0;
    }
}
