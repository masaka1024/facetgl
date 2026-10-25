// Physics parity check: does the TypeScript engine (src/physics/engine) still give exactly the
// same results as the C# engine (mmd2gltf-cs-physics)? See README.md.
//
//   npm run parity -- --model <model.pmx> [--engine <mmd2gltf-cs-physics folder>] [--quick]
//
// Builds the C# dumper against the engine source, bundles the TypeScript runner, runs both on the
// same scenario in several configurations and compares every rigid body bit for bit.
import { spawnSync } from 'node:child_process';
import { existsSync, mkdirSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { compareDumps } from './compare.mjs';

const here = dirname(fileURLToPath(import.meta.url));
const repo = resolve(here, '../..');
const out = join(here, '.out');

// --- arguments ---
const argv = process.argv.slice(2);
const opt = (name) => {
  const i = argv.indexOf(name);
  return i >= 0 ? argv[i + 1] : undefined;
};
const model = opt('--model') ?? process.env.PARITY_MODEL;
const engine = resolve(opt('--engine') ?? process.env.ENGINE_DIR ?? join(repo, '..', 'mmd2gltf-cs-physics'));
const quick = argv.includes('--quick');
if (!model) {
  console.error('使い方: npm run parity -- --model <モデル.pmx> [--engine <mmd2gltf-cs-physics のフォルダ>] [--quick]');
  process.exit(2);
}
if (!existsSync(model)) fail(`モデルが見つかりません: ${model}`);
if (!existsSync(join(engine, 'Assets', 'MmdPhysics', 'Core'))) {
  fail(`C# エンジンのフォルダが見つかりません: ${engine}\n  --engine で mmd2gltf-cs-physics を clone したフォルダを指定してください`);
}
mkdirSync(out, { recursive: true });

function fail(message) {
  console.error(`\n✗ ${message}`);
  process.exit(2);
}

function run(command, args, env = {}) {
  const r = spawnSync(command, args, { encoding: 'utf8', env: { ...process.env, ...env } });
  if (r.status !== 0) fail(`${command} ${args.join(' ')}\n${r.stdout ?? ''}${r.stderr ?? ''}${r.error ?? ''}`);
  return r.stdout.trim();
}

// --- 1. build the C# dumper against the engine source ---
console.log(`C# エンジン: ${engine}`);
console.log('C# ダンパーをビルド中…');
run('dotnet', ['build', join(here, 'dumper', 'Dumper.csproj'), '-c', 'Release', '-nologo', '-v', 'q', `-p:EngineDir=${engine}`, '-o', join(out, 'dumper')]);
const dumper = join(out, 'dumper', 'Dumper.dll');

// --- 2. bundle the TypeScript runner ---
console.log('TypeScript ランナーを準備中…');
const esbuild = await import('esbuild');
await esbuild.build({
  entryPoints: [join(here, 'runner.ts')],
  bundle: true,
  platform: 'node',
  format: 'esm',
  target: 'node18',
  outfile: join(out, 'runner.mjs'),
  logLevel: 'warning',
});
const runner = join(out, 'runner.mjs');

// --- 3. cases ---
const dynamic = JSON.parse(run('node', [runner, model, '--list-dynamic']));
const perturbBodies = dynamic.length ? [dynamic[0], dynamic[Math.floor(dynamic.length / 2)]] : [];
const scen = join(out, 'scen.bin');

const csDumps = new Map();
function csDump(cfg, perturb) {
  const key = `cs_${cfg}_${perturb}`;
  if (!csDumps.has(key)) {
    const file = join(out, `${key}.bin`);
    const args = [dumper, model, file, cfg === '0' && perturb < 0 ? scen : '', String(perturb)];
    const log = run('dotnet', args, { CFG: cfg });
    csDumps.set(key, { file, log });
  }
  return csDumps.get(key);
}
csDump('0', -1); // writes the scenario file first

const cases = [{ name: '既定設定', cfg: '0', perturb: -1 }];
if (!quick) {
  cases.push(
    { name: '既定設定 (TypeScript のみ)', cfg: '0', perturb: -1, nowasm: true },
    { name: '既定外の設定1', cfg: '1', perturb: -1 },
    { name: '既定外の設定1 (TypeScript のみ)', cfg: '1', perturb: -1, nowasm: true },
    { name: '既定外の設定2', cfg: '2', perturb: -1 },
    ...perturbBodies.map((b) => ({ name: `剛体${b} を1ulpずらした開始`, cfg: '0', perturb: b })),
  );
}

console.log('\n比較中…\n');
let failed = 0;
let speed = '';
for (const c of cases) {
  const cs = csDump(c.cfg, c.perturb);
  const tsFile = join(out, `ts_${c.cfg}_${c.perturb}${c.nowasm ? '_nowasm' : ''}.bin`);
  const tsLog = run('node', [runner, model, scen, tsFile, String(c.perturb)], { CFG: c.cfg, NOWASM: c.nowasm ? '1' : '0' });
  const r = compareDumps(cs.file, tsFile);
  const ok = r.firstDiffFrame < 0;
  if (!ok) failed++;
  const last = r.rows[r.rows.length - 1];
  console.log(
    `${ok ? '✓' : '✗'} ${c.name.padEnd(28)} ` +
      (ok ? `${r.frames} フレームすべてビット一致` : `フレーム ${r.firstDiffFrame} から不一致 (最終: ${last.differing} 剛体, 位置差 ${last.maxPos.toExponential(2)})`),
  );
  if (c.name === '既定設定') speed = `剛体 ${r.bodies} / 1/60 秒あたり: C# ${cs.log.split('ms/frame=')[1]} ms, TypeScript ${tsLog.split('ms/frame=')[1]} ms`;
}

// Chaos control: how far a 1-ulp change grows in the C# engine itself (for reading a mismatch)
if (!quick && perturbBodies.length) {
  const r = compareDumps(csDump('0', -1).file, csDump('0', perturbBodies[0]).file);
  const last = r.rows[r.rows.length - 1];
  console.log(`\n参考: C# 自身で1ulpずらすと、最終フレームで ${last.differing} 剛体が最大 ${last.maxPos.toExponential(2)} ずれる (系の敏感さの目安)`);
}
console.log(`\n${speed}`);

// Engine harness scenarios (informational: the C# harness gives the same values)
if (!quick) {
  await esbuild.build({ entryPoints: [join(here, 'harness.ts')], bundle: true, platform: 'node', format: 'esm', target: 'node18', outfile: join(out, 'harness.mjs'), logLevel: 'warning' });
  const h = spawnSync('node', [join(out, 'harness.mjs'), model], { encoding: 'utf8' });
  console.log(`\n--- 参考: エンジン付属テストの TypeScript 版 (C# 版 tools/compilecheck と同じ値になるはず) ---\n${(h.stdout ?? '').trim()}`);
}
console.log(failed === 0 ? '\n=== すべて一致 ===' : `\n=== ${failed} 件で不一致 ===`);
process.exit(failed === 0 ? 0 : 1);
