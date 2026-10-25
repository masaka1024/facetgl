// ===========================================================================
// MMD physics engine – loader for the float32 WebAssembly velocity-solver kernel (wasm/solver.c).
// The kernel performs the same float32 operations as the TypeScript solver (bit-identical); it is
// only faster because WebAssembly has native binary32 arithmetic. If WebAssembly is unavailable the
// world keeps using the TypeScript solver.
// ===========================================================================

import { SOLVER_WASM_BASE64 } from './wasm/solverWasm';

export interface SolverKernel {
  memory: WebAssembly.Memory;
  solve: (bodies: number, rows: number, nRows: number, contacts: number, nContacts: number, contactIters: number, jointIters: number, flags: number) => void;
  broadphase: (n: number, bb: number, org: number, sphR: number, thr: number, okBits: number, candBits: number, order: number, pack: number, axis: number, prefilter: number, slack: number, outPairs: number) => number;
  prepare: (nJoints: number, joints: number, bodies: number, params: number, iters: number, rows: number, rowCounts: number) => number;
  heapBase: number;
  f32: Float32Array;
  i32: Int32Array;
  /** Makes sure the linear memory holds `bytes` bytes after heapBase; refreshes the views. */
  reserve(bytes: number): void;
}

let kernel: SolverKernel | null | undefined;
/** The kernel calls back into these for atan2 / asin, so the results are exactly the TypeScript ones. */
const IMPORTS = { m: { atan2: Math.atan2, asin: Math.asin } };
let pending = false;

function decode(b64: string): Uint8Array {
  if (typeof atob === 'function') {
    const s = atob(b64);
    const out = new Uint8Array(s.length);
    for (let i = 0; i < s.length; i++) out[i] = s.charCodeAt(i);
    return out;
  }
  const B = (globalThis as unknown as { Buffer?: { from(s: string, e: string): Uint8Array } }).Buffer;
  if (B) return new Uint8Array(B.from(b64, 'base64'));
  throw new Error('no base64 decoder');
}

function makeKernel(inst: WebAssembly.Instance): SolverKernel {
  const ex = inst.exports as unknown as {
    memory: WebAssembly.Memory;
    solve: SolverKernel['solve'];
    broadphase: SolverKernel['broadphase'];
    prepare: SolverKernel['prepare'];
    heapBase: () => number;
  };
  const k: SolverKernel = {
    memory: ex.memory,
    solve: ex.solve,
    broadphase: ex.broadphase,
    prepare: ex.prepare,
    heapBase: (ex.heapBase() + 15) & ~15,
    f32: new Float32Array(ex.memory.buffer),
    i32: new Int32Array(ex.memory.buffer),
    reserve(bytes: number) {
      const need = k.heapBase + bytes;
      const have = k.memory.buffer.byteLength;
      if (need > have) k.memory.grow(Math.ceil((need - have) / 65536));
      if (k.f32.buffer !== k.memory.buffer) {
        k.f32 = new Float32Array(k.memory.buffer);
        k.i32 = new Int32Array(k.memory.buffer);
      }
    },
  };
  return k;
}

/** Disable the kernel (forces the TypeScript solver), e.g. for A/B checks. */
export const solverKernelOptions = { disabled: false };

/** The kernel, or null while it is unavailable (still compiling asynchronously, or no WebAssembly). */
export function getSolverKernel(): SolverKernel | null {
  if (solverKernelOptions.disabled) return null;
  if (kernel !== undefined) return kernel;
  if (pending) return null;
  if (typeof WebAssembly === 'undefined') return (kernel = null);
  const bytes = decode(SOLVER_WASM_BASE64);
  try {
    // Synchronous compile is allowed for small modules (< 4 KB on browser main threads).
    kernel = makeKernel(new WebAssembly.Instance(new WebAssembly.Module(bytes as BufferSource), IMPORTS));
  } catch {
    if (!pending) {
      pending = true;
      WebAssembly.instantiate(bytes as BufferSource, IMPORTS)
        .then((r) => (kernel = makeKernel(r.instance)))
        .catch(() => (kernel = null));
    }
    return null;
  }
  return kernel;
}

/**
 * Compiles the kernel asynchronously (browsers refuse synchronous compilation of modules > 4 KB on the
 * main thread). Optional: until it resolves the world uses the TypeScript solver, which gives the same bits.
 * Resolves to true when the kernel is available.
 */
export async function initSolverKernel(): Promise<boolean> {
  if (kernel !== undefined) return kernel !== null;
  if (typeof WebAssembly === 'undefined') {
    kernel = null;
    return false;
  }
  pending = true;
  try {
    const r = await WebAssembly.instantiate(decode(SOLVER_WASM_BASE64) as BufferSource, IMPORTS);
    kernel = makeKernel(r.instance);
  } catch {
    kernel = null;
  }
  return kernel !== null;
}
