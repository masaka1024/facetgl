// Compares two parity dumps bit for bit (see README.md).
//   node compare.mjs <a.bin> <b.bin>        prints the first differing frame and a summary table
// Also used by check.mjs (compareDumps).
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

const MAGIC = 0x504d4450;

function load(path) {
  const b = readFileSync(path);
  const buffer = b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength);
  const [magic, bodies, frames, rec] = new Int32Array(buffer, 0, 4);
  if (magic !== MAGIC) throw new Error(`${path}: not a parity dump`);
  return { bodies, frames, rec, f32: new Float32Array(buffer, 16), u32: new Uint32Array(buffer, 16) };
}

// Angle between two (possibly unnormalized) quaternions, in radians
function quatAngle(a, b) {
  const na = Math.hypot(...a), nb = Math.hypot(...b);
  const sign = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] < 0 ? -1 : 1;
  let dm = 0, dp = 0;
  for (let c = 0; c < 4; c++) {
    const x = a[c] / na, y = (sign * b[c]) / nb;
    dm += (x - y) ** 2;
    dp += (x + y) ** 2;
  }
  return 4 * Math.atan2(Math.sqrt(dm), Math.sqrt(dp));
}

export function compareDumps(aPath, bPath) {
  const A = load(aPath), B = load(bPath);
  if (A.bodies !== B.bodies || A.rec !== B.rec) throw new Error(`different layouts (${A.bodies} / ${B.bodies} bodies)`);
  const frames = Math.min(A.frames, B.frames);
  let firstDiffFrame = -1;
  const rows = [];
  for (let k = 0; k < frames; k++) {
    let differing = 0, maxPos = 0, maxAngle = 0;
    for (let i = 0; i < A.bodies; i++) {
      const p = (k * A.bodies + i) * A.rec;
      let diff = false;
      for (let c = 0; c < A.rec; c++) if (A.u32[p + c] !== B.u32[p + c]) diff = true;
      if (!diff) continue;
      differing++;
      const dp = Math.hypot(A.f32[p] - B.f32[p], A.f32[p + 1] - B.f32[p + 1], A.f32[p + 2] - B.f32[p + 2]);
      if (!(dp <= maxPos)) maxPos = dp;
      const ang = quatAngle([...A.f32.subarray(p + 3, p + 7)], [...B.f32.subarray(p + 3, p + 7)]);
      if (!(ang <= maxAngle)) maxAngle = ang;
    }
    if (firstDiffFrame < 0 && differing > 0) firstDiffFrame = k;
    rows.push({ frame: k, differing, maxPos, maxAngle });
  }
  return { bodies: A.bodies, frames, firstDiffFrame, rows };
}

if (process.argv[1] === fileURLToPath(import.meta.url)) {
  const [a, b] = process.argv.slice(2);
  const r = compareDumps(a, b);
  console.log(`bodies=${r.bodies} frames=${r.frames} firstDiffFrame=${r.firstDiffFrame}${r.firstDiffFrame < 0 ? ' (bit-identical)' : ''}`);
  const show = new Set([0, 59, 60, 120, 180, 239, 240, 300, r.frames - 1, r.firstDiffFrame, r.firstDiffFrame + 1]);
  for (const row of r.rows) {
    if (!show.has(row.frame)) continue;
    console.log(`  f${row.frame}\tdifferingBodies=${row.differing}\tmax|Δpos|=${row.maxPos.toExponential(2)}\tmaxΔangle=${row.maxAngle.toExponential(2)} rad`);
  }
  process.exit(r.firstDiffFrame < 0 ? 0 : 1);
}
