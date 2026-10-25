// Physics parity runner (TypeScript side). See README.md.
//
// Same scenario and dump format as dumper/Dumper.cs, using the engine port in src/physics/engine.
//
//   runner <model.pmx> <scenario.bin> <out.bin> [perturbBody]
//   runner <model.pmx> --list-dynamic        prints the indices of the dynamic bodies (JSON)
//   env CFG = 0 / 1 / 2 (same switch sets as the dumper), NOWASM=1 = TypeScript solver only
import { readFileSync, writeFileSync } from 'node:fs';
import { performance } from 'node:perf_hooks';
import {
  CapsuleShape, CollisionShape, GjkEpa, Joint, PersistentManifold, PhysicsWorld, PmxPhysicsBuilder, Quat, RigidTransform, Vec3,
  initSolverKernel, readPmx, solverKernelOptions,
} from '../../src/physics/engine/index';

const MAGIC = 0x504d4450;
const REC = 13;

const [pmxPath, arg2, outPath, perturbArg] = process.argv.slice(2);
const read = (path: string) => {
  const b = readFileSync(path);
  return b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength);
};

if (process.env.NOWASM === '1') solverKernelOptions.disabled = true;
else await initSolverKernel();

const cfg = process.env.CFG ?? '0';
if (cfg === '2') {
  CollisionShape.bulletShapeMargin = true;
  CapsuleShape.inertiaMargin = 0;
}
const model = readPmx(read(pmxPath));
PmxPhysicsBuilder.maxDynamicMass = 0;
const b = PmxPhysicsBuilder.build(model);
const w = b.world;

if (arg2 === '--list-dynamic') {
  console.log(JSON.stringify(w.bodies.flatMap((body, i) => (body.isStaticOrKinematic ? [] : [i]))));
  process.exit(0);
}

w.gravity = new Vec3(0, -98, 0);
w.solverIterations = 10;
w.subSteps = 2;
w.fixedTimeStep = 1 / 60;
w.useSplitImpulse = false;
w.useJointSplitImpulse = false;
w.enableSleeping = false;
w.useBroadphasePrefilter = true;
if (cfg === '1') {
  w.useSplitImpulse = true; w.useJointSplitImpulse = true; Joint.linearLeverMode = 2; PersistentManifold.bulletManifoldPoints = false;
  Joint.springAsMotorRow = false; w.contactRhsBullet = false; w.frictionVelocityAligned = false; w.frictionCombineMultiply = false;
  PhysicsWorld.bulletRotationIntegration = false; w.enableSleeping = true; w.useJointWarmStart = true; w.useJointWarmStartAngular = true; w.solveJointsFirst = true;
}
if (cfg === '2') {
  Joint.bulletLimitRowGating = false; Joint.angularMixedAxes = false; Joint.bulletAngleConvention = false; Joint.linearLeverMode = 0;
  w.contactPoolOrder = true; GjkEpa.bulletContactThreshold = false; w.jointVelocityIterations = 15; PersistentManifold.symmetricBreakingDistance = false;
}

const bind = model.bonePositions;
b.resetBodiesToBonePoseFk((i) => new RigidTransform(Quat.Identity, bind[i]));
const perturb = perturbArg ? parseInt(perturbArg) : -1;
if (perturb >= 0) {
  const body = w.bodies[perturb];
  const t = body.worldTransform;
  const x = new Float32Array([t.origin.x]);
  new Int32Array(x.buffer)[0]++;
  body.worldTransform = new RigidTransform(t.rotation, new Vec3(x[0], t.origin.y, t.origin.z));
}

const scen = new Float32Array(read(arg2));
const frames = scen.length / 7;
const root = bind[0];
const nBodies = w.bodies.length;
const dump = new Float32Array(4 + frames * nBodies * REC);
new Int32Array(dump.buffer, 0, 4).set([MAGIC, nBodies, frames, REC]);
let steady = 0, steadyN = 0;
for (let k = 0; k < frames; k++) {
  const q = new Quat(scen[k * 7], scen[k * 7 + 1], scen[k * 7 + 2], scen[k * 7 + 3]);
  const o = new Vec3(scen[k * 7 + 4], scen[k * 7 + 5], scen[k * 7 + 6]);
  const bw = (i: number) => new RigidTransform(q, q.rotate(bind[i].sub(root)).add(root).add(o));
  const t0 = performance.now();
  b.applyKinematicTargets(bw);
  w.stepSimulation(1 / 60);
  if (k >= 20) {
    steady += performance.now() - t0;
    steadyN++;
  }
  for (let i = 0; i < nBodies; i++) {
    const bd = w.bodies[i];
    const p = 4 + (k * nBodies + i) * REC;
    const tr = bd.worldTransform, lv = bd.linearVelocity, av = bd.angularVelocity;
    dump.set([tr.origin.x, tr.origin.y, tr.origin.z, tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w, lv.x, lv.y, lv.z, av.x, av.y, av.z], p);
  }
}
writeFileSync(outPath, new Uint8Array(dump.buffer));
console.log(`bodies=${nBodies} joints=${w.joints.length} frames=${frames} ms/frame=${(steady / steadyN).toFixed(2)}`);
