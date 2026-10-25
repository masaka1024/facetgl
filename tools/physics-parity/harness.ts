// Engine harness scenarios (port of the C# engine's tools/compilecheck/Harness.cs, Unity-free parts).
// Synthetic scenes plus smoke checks on a model; values print like C# "R" so they can be compared
// with the C# harness output. Informational: run by check.mjs, does not decide its result.
import { readFileSync } from 'node:fs';
import { performance } from 'node:perf_hooks';
import {
  f, readPmx, PmxPhysicsBuilder, PhysicsWorld, RigidBody, PhysicsMode, SphereShape, BoxShape, CollisionShape,
  Joint, JointType, Quat, RigidTransform, Vec3, GjkEpa,
} from '../../src/physics/engine/index';

const R = (v: number) => { if (!Number.isFinite(v)) return String(v); for (let p = 1; p <= 9; p++) { const s = Number(v.toPrecision(p)); if (f(s) === v) return String(s); } return String(v); };
let fails = 0;
const check = (name: string, ok: boolean, detail: string) => { console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${name}  (${detail})`); if (!ok) fails++; };
const inRange = (v: number, lo: number, hi: number) => !Number.isNaN(v) && v > lo && v < hi;
const pmxPath = process.argv[2];
const loadModel = () => { const b = readFileSync(pmxPath); return readPmx(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength)); };

function world60() { const w = new PhysicsWorld(); w.gravity = new Vec3(0, f(-9.8), 0); w.subSteps = 2; w.fixedTimeStep = 1 / 60; w.solverIterations = 10; return w; }
function dyn(s: CollisionShape, m: number, p: Vec3) { const b = new RigidBody(s); b.mode = PhysicsMode.Dynamic; b.group = 0; b.collisionMask = 0xffff; b.setMassProps(m); b.worldTransform = new RigidTransform(Quat.Identity, p); return b; }
function kin(s: CollisionShape, p: Vec3) { const b = new RigidBody(s); b.mode = PhysicsMode.BoneFollow; b.group = 0; b.collisionMask = 0xffff; b.worldTransform = new RigidTransform(Quat.Identity, p); b.setMassProps(0); return b; }

console.log('=== TS port harness ===');
{ const w = world60(); const b = dyn(new SphereShape(0.5), 1, new Vec3(0, 10, 0)); w.addBody(b); for (let i = 0; i < 60; i++) w.stepSimulation(f(1 / 60));
  check('FreeFall (1s後 y≈5.1)', inRange(b.worldTransform.origin.y, 4, 6), `y=${R(b.worldTransform.origin.y)}`); }
{ const w = world60(); const fl = kin(new BoxShape(new Vec3(20, 1, 20)), new Vec3(0, -1, 0)); w.addBody(fl); const b = dyn(new SphereShape(0.5), 1, new Vec3(0, 3, 0)); b.friction = 0.5; w.addBody(b);
  for (let i = 0; i < 300; i++) w.stepSimulation(f(1 / 60)); const y = b.worldTransform.origin.y; check('GroundRest (真接触 y≈0.50)', inRange(y, f(0.49), f(0.51)), `y=${R(y)}`); }
{ const w = world60(); const an = kin(new BoxShape(Vec3.of(0.3, 0.3, 0.3)), new Vec3(0, 5, 0)); w.addBody(an); const h = dyn(new BoxShape(Vec3.of(0.3, 0.3, 0.3)), 1, new Vec3(0, 4, 0)); w.addBody(h);
  w.addJoint(Joint.fromPmx(JointType.Generic6Dof, an, h, new RigidTransform(Quat.Identity, new Vec3(0, 4.5, 0)), Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero, Vec3.Zero));
  for (let i = 0; i < 300; i++) w.stepSimulation(f(1 / 60)); check('JointHold (ロック保持 y≈4.0)', inRange(h.worldTransform.origin.y, 3.5, 4.5), `y=${R(h.worldTransform.origin.y)}`); }
{ const w = world60(); const an = kin(new SphereShape(0.2), new Vec3(0, 5, 0)); w.addBody(an); const bob = dyn(new SphereShape(0.2), 1, new Vec3(0, 4, 0)); w.addBody(bob);
  w.addJoint(Joint.fromPmx(JointType.Spring6Dof, an, bob, new RigidTransform(Quat.Identity, new Vec3(0, 4.5, 0)), new Vec3(-2, -2, -2), new Vec3(2, 2, 2), Vec3.splat(1), Vec3.splat(-1), new Vec3(50, 50, 50), Vec3.Zero));
  let mn = 999; for (let i = 0; i < 600; i++) { w.stepSimulation(f(1 / 60)); const y = bob.worldTransform.origin.y; if (i > 200) mn = Math.min(mn, y); }
  check('SpringPendulum (発散せず有限)', !Number.isNaN(mn) && mn > 0 && mn < 5, `minY=${R(mn)}`); }

// modelA smoke (default world: 30 Hz, 4 substeps, no reset)
{ GjkEpa.epaIterCapHits = 0; GjkEpa.epaFaceCapHits = 0;
  const b = PmxPhysicsBuilder.build(loadModel()); const w = b.world; w.gravity = new Vec3(0, -98, 0);
  const dynB = w.bodies.filter((r) => !r.isStaticOrKinematic); const initPos = dynB.map((r) => r.worldTransform.origin);
  let badNum = false, maxSpeed = 0, stepMax = 0, maxDrift = 0, driftName = '';
  const bad = (x: number) => !Number.isFinite(x);
  for (let i = 0; i < 300; i++) {
    const t0 = performance.now(); w.stepSimulation(w.fixedTimeStep); const ms = performance.now() - t0; if (i >= 5) stepMax = Math.max(stepMax, ms);
    for (let k = 0; k < dynB.length; k++) { const r = dynB[k]; const o = r.worldTransform.origin, v = r.linearVelocity;
      if (bad(o.x) || bad(o.y) || bad(o.z) || bad(v.x) || bad(v.y) || bad(v.z)) badNum = true;
      const sp = v.length; if (sp > maxSpeed) maxSpeed = sp; const dr = o.sub(initPos[k]).length; if (dr > maxDrift) { maxDrift = dr; driftName = r.name; } }
  }
  const epaHits = GjkEpa.epaIterCapHits + GjkEpa.epaFaceCapHits;
  check('modelA.pmx スモーク (NaN/爆発/EPA暴走/遅延なし)', !badNum && maxSpeed <= 100 && epaHits === 0, `NaN/Inf=${badNum} maxSpeed=${R(maxSpeed)}(<=100) EPAhit=${epaHits}(=0) stepMax=${stepMax.toFixed(2)}ms(not gated in TS)`);
  check('modelA.pmx 静止ドリフト (爆散番人, <15)', maxDrift < 15, `maxDrift=${R(maxDrift)} (最大: ${driftName})`); }
// static push regression
{ const b = PmxPhysicsBuilder.build(loadModel()); const w = b.world; w.gravity = Vec3.Zero; w.subSteps = 1; w.fixedTimeStep = 1 / 30;
  const dump: { a: string; b: string; dist: number; ni: number }[] = []; w.debugContacts = dump; w.stepSimulation(w.fixedTimeStep);
  const bad = dump.filter((x) => x.dist > 0 && Math.abs(x.ni) > f(1e-8)); bad.sort((x, y) => Math.abs(y.ni) - Math.abs(x.ni));
  check('静止押し出し回帰 (非貫入インパルス0件)', bad.length === 0, `件数=${bad.length}  例:${bad.slice(0, 3).map((x) => `${x.a}×${x.b}(d=${R(x.dist)},ni=${R(x.ni)})`).join(', ')}`); }
// kinematic interpolation
for (const [fts, sub] of [[f(1 / 30), 1], [f(1 / 30), 2], [f(1 / 30), 4], [f(1 / 60), 1], [f(1 / 120), 1]] as const) {
  const w = new PhysicsWorld(); w.gravity = Vec3.Zero; w.solverIterations = 10; w.fixedTimeStep = fts; w.subSteps = sub;
  const k = new RigidBody(new BoxShape(Vec3.of(0.5, 0.5, 0.5))); k.mode = PhysicsMode.BoneFollow; k.setMassProps(0); k.worldTransform = new RigidTransform(Quat.Identity, Vec3.Zero); w.addBody(k);
  k.kinematicTarget = new RigidTransform(Quat.Identity, new Vec3(1, 0, 0)); w.stepSimulation(f(1 / 30));
  const vx = k.linearVelocity.x; check(`Kinematic補間 (FTS=1/${R(f(1 / fts))} Sub${sub}) 残存velx≈30`, Math.abs(f(vx - 30)) < f(1e-2), `velx=${R(vx)} x=${R(k.worldTransform.origin.x)}`);
}
// drive path unification
{ const b = PmxPhysicsBuilder.build(loadModel());
  const pose = (i: number) => new RigidTransform(new Quat(f(f(Math.sin(i * 0.1)) * f(0.3)), f(0.1), f(f(Math.cos(i * 0.2)) * f(0.2)), 1).normalized, new Vec3(f(i * f(0.01)), f(1 + f(i * f(0.02))), f(-i * f(0.005))));
  b.applyKinematicTargets(pose); let maxErr = 0, nFollow = 0;
  for (const link of b.boneLinks) { if (link.mode !== PhysicsMode.BoneFollow || link.boneIndex < 0) continue; nFollow++;
    const e = pose(link.boneIndex).mul(link.bodyOffsetFromBone); const g = link.body.kinematicTarget;
    maxErr = Math.max(maxErr, g.origin.sub(e.origin).length);
    const gr = g.rotation, er = e.rotation; const qd = Math.abs(f(f(f(f(gr.x * er.x) + f(gr.y * er.y)) + f(gr.z * er.z)) + f(gr.w * er.w)));
    maxErr = Math.max(maxErr, Math.abs(f(1 - qd))); }
  check('駆動式統一 (KinematicTarget=bw*offset 共通ヘルパ)', nFollow > 0 && maxErr < f(1e-6), `BoneFollow=${nFollow} maxErr=${R(maxErr)}`); }
// collision pair cache
{ const w = new PhysicsWorld(); const mk = (x: number) => { const b = new RigidBody(new SphereShape(0.5)); b.mode = PhysicsMode.Dynamic; b.group = 0; b.collisionMask = 0xffff; b.setMassProps(1); b.worldTransform = new RigidTransform(Quat.Identity, new Vec3(x, 0, 0)); return b; };
  w.addBody(mk(0)); w.addBody(mk(1)); const p2 = w.debugCollisionPairCount; w.addBody(mk(2)); const p3 = w.debugCollisionPairCount;
  w.bodies[2].collisionMask = 0; w.invalidateCollisionPairs(); const p3m = w.debugCollisionPairCount;
  check('候補ペアキャッシュ (追加で増える/無効化で反映)', p2 === 1 && p3 === 3 && p3m === 1, `2体=${p2}(期待1) 3体=${p3}(期待3) マスク0後=${p3m}(期待1)`); }
console.log(fails === 0 ? '\n(参考) 全 PASS' : `\n(参考) ${fails} 件 FAIL — モデルによっては C# 版でも同じ項目が FAIL します。C# 版の結果と見比べてください`);
