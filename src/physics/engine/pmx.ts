// ===========================================================================
// MMD physics engine (TypeScript port) – PMX physics data + reader
// Port of PmxPhysicsData.cs + PmxReader.cs. Reads PMX 2.0 / 2.1 (UTF-16LE or UTF-8 text,
// index sizes 1/2/4) and keeps bones (name / position / parent), rigid bodies, joints and
// soft bodies; everything else is skipped exactly.
// ===========================================================================

import { f, Vec3 } from './math';

export class PmxRigidBody {
  name = '';
  nameEn = '';
  boneIndex = -1;
  group = 0;
  nonCollisionGroup = 0;
  /** 0: sphere, 1: box, 2: capsule */
  shapeType = 0;
  size: Vec3 = Vec3.Zero;
  position: Vec3 = Vec3.Zero;
  /** radians */
  rotation: Vec3 = Vec3.Zero;
  mass = 0;
  linearDamping = 0;
  angularDamping = 0;
  restitution = 0;
  friction = 0;
  /** 0: bone follow, 1: physics, 2: physics + bone alignment */
  physicsMode = 0;
}

export class PmxJoint {
  name = '';
  nameEn = '';
  jointType = 0;
  rigidBodyAIndex = -1;
  rigidBodyBIndex = -1;
  position: Vec3 = Vec3.Zero;
  rotation: Vec3 = Vec3.Zero;
  linearLowerLimit: Vec3 = Vec3.Zero;
  linearUpperLimit: Vec3 = Vec3.Zero;
  angularLowerLimit: Vec3 = Vec3.Zero;
  angularUpperLimit: Vec3 = Vec3.Zero;
  springLinear: Vec3 = Vec3.Zero;
  springAngular: Vec3 = Vec3.Zero;
}

export interface PmxSoftBodyAnchor {
  rigidBodyIndex: number;
  vertexIndex: number;
  nearMode: boolean;
}

export class PmxSoftBody {
  name = '';
  nameEn = '';
  shape = 0;
  materialIndex = -1;
  group = 0;
  nonCollisionGroup = 0;
  flags = 0;
  bLinkDistance = 0;
  clusterCount = 0;
  totalMass = 0;
  collisionMargin = 0;
  aeroModel = 0;
  /** config VCF, DP, DG, LF, PR, VC, DF, MT, CHR, KHR, SHR, AHR */
  config: number[] = [];
  /** cluster SRHR_CL, SKHR_CL, SSHR_CL, SR_SPLT_CL, SK_SPLT_CL, SS_SPLT_CL */
  cluster: number[] = [];
  /** iteration V_IT, P_IT, D_IT, C_IT */
  iteration: number[] = [];
  /** material LST, AST, VST */
  material: number[] = [];
  readonly anchors: PmxSoftBodyAnchor[] = [];
  readonly pinVertices: number[] = [];
}

/** Physics-relevant data of one PMX model. */
export class PmxPhysicsModel {
  modelName = '';
  modelNameEn = '';
  version = 0;

  readonly rigidBodies: PmxRigidBody[] = [];
  readonly joints: PmxJoint[] = [];
  readonly softBodies: PmxSoftBody[] = [];

  readonly boneNames: string[] = [];
  /** Bone positions in MMD model coordinates. */
  readonly bonePositions: Vec3[] = [];
  /** Parent bone index (-1 = root). */
  readonly boneParents: number[] = [];
  readonly boneDeformLayers: number[] = [];
}

class Reader {
  private pos = 0;
  private readonly view: DataView;
  private readonly bytes: Uint8Array;
  encoding = 0;
  addUv = 0;
  vertexIndexSize = 0;
  textureIndexSize = 0;
  materialIndexSize = 0;
  boneIndexSize = 0;
  morphIndexSize = 0;
  rigidIndexSize = 0;
  private static readonly utf16 = new TextDecoder('utf-16le');
  private static readonly utf8 = new TextDecoder('utf-8');

  constructor(buffer: ArrayBuffer) {
    this.view = new DataView(buffer);
    this.bytes = new Uint8Array(buffer);
  }

  get position() {
    return this.pos;
  }
  get length() {
    return this.bytes.length;
  }

  private need(n: number) {
    if (n < 0 || this.pos + n > this.bytes.length) throw new RangeError('PMX: unexpected end of data');
  }
  u8() {
    this.need(1);
    return this.view.getUint8(this.pos++);
  }
  i8() {
    this.need(1);
    return this.view.getInt8(this.pos++);
  }
  u16() {
    this.need(2);
    const v = this.view.getUint16(this.pos, true);
    this.pos += 2;
    return v;
  }
  i16() {
    this.need(2);
    const v = this.view.getInt16(this.pos, true);
    this.pos += 2;
    return v;
  }
  i32() {
    this.need(4);
    const v = this.view.getInt32(this.pos, true);
    this.pos += 4;
    return v;
  }
  f32() {
    this.need(4);
    const v = this.view.getFloat32(this.pos, true);
    this.pos += 4;
    return v;
  }
  bytesN(n: number) {
    this.need(n);
    const b = this.bytes.subarray(this.pos, this.pos + n);
    this.pos += n;
    return b;
  }
  skip(n: number) {
    // BinaryReader.BaseStream.Seek allows seeking past the end; a later read then fails.
    this.pos += n;
  }
  vec3() {
    const x = this.f32(), y = this.f32(), z = this.f32();
    return new Vec3(x, y, z);
  }
  text() {
    const len = this.i32();
    if (len <= 0) return '';
    const b = this.bytesN(len);
    return this.encoding === 0 ? Reader.utf16.decode(b) : Reader.utf8.decode(b);
  }
  index(size: number) {
    switch (size) {
      case 1: return this.i8();
      case 2: return this.i16();
      case 4: return this.i32();
      default: throw new Error(`PMX: invalid index size ${size}`);
    }
  }
  vertexIndex() {
    switch (this.vertexIndexSize) {
      case 1: return this.u8();
      case 2: return this.u16();
      case 4: return this.i32();
      default: throw new Error('PMX: invalid vertex index size');
    }
  }
}

const F_2_1 = f(2.1);

/** Reads the physics data of a PMX 2.0 / 2.1 file. */
export function readPmx(buffer: ArrayBuffer): PmxPhysicsModel {
  const r = new Reader(buffer);
  const model = new PmxPhysicsModel();

  // header
  const magic = r.bytesN(4);
  if (magic[0] !== 0x50 || magic[1] !== 0x4d || magic[2] !== 0x58 || magic[3] !== 0x20)
    throw new Error('PMX: magic number mismatch');
  model.version = r.f32();
  const flagCount = r.u8();
  const flags = r.bytesN(flagCount);
  if (flags.length < 8) throw new RangeError('PMX: header flags too short');
  r.encoding = flags[0];
  r.addUv = flags[1];
  r.vertexIndexSize = flags[2];
  r.textureIndexSize = flags[3];
  r.materialIndexSize = flags[4];
  r.boneIndexSize = flags[5];
  r.morphIndexSize = flags[6];
  r.rigidIndexSize = flags[7];

  model.modelName = r.text();
  model.modelNameEn = r.text();
  r.text();
  r.text();

  // vertices
  {
    const count = r.i32();
    for (let i = 0; i < count; i++) {
      r.skip(12 + 12 + 8);
      r.skip(16 * r.addUv);
      const weightType = r.u8();
      switch (weightType) {
        case 0: r.skip(r.boneIndexSize); break;
        case 1: r.skip(r.boneIndexSize * 2 + 4); break;
        case 2: r.skip(r.boneIndexSize * 4 + 16); break;
        case 3: r.skip(r.boneIndexSize * 2 + 4 + 36); break;
        case 4: r.skip(r.boneIndexSize * 4 + 16); break;
        default: throw new Error(`PMX: invalid weight type ${weightType}`);
      }
      r.skip(4);
    }
  }
  // faces
  {
    const count = r.i32();
    r.skip(count * r.vertexIndexSize);
  }
  // textures
  {
    const count = r.i32();
    for (let i = 0; i < count; i++) r.text();
  }
  // materials
  {
    const count = r.i32();
    for (let i = 0; i < count; i++) {
      r.text();
      r.text();
      r.skip(16 + 12 + 4 + 12);
      r.skip(1);
      r.skip(16 + 4);
      r.skip(r.textureIndexSize * 2);
      r.skip(1);
      const sharedToon = r.u8();
      r.skip(sharedToon === 0 ? r.textureIndexSize : 1);
      r.text();
      r.skip(4);
    }
  }
  // bones
  {
    const count = r.i32();
    for (let i = 0; i < count; i++) {
      model.boneNames.push(r.text());
      r.text();
      model.bonePositions.push(r.vec3());
      model.boneParents.push(r.index(r.boneIndexSize));
      model.boneDeformLayers.push(r.i32());
      const bflags = r.u16();
      const connectByBone = (bflags & 0x0001) !== 0;
      r.skip(connectByBone ? r.boneIndexSize : 12);
      if ((bflags & 0x0100) !== 0 || (bflags & 0x0200) !== 0) r.skip(r.boneIndexSize + 4);
      if ((bflags & 0x0400) !== 0) r.skip(12);
      if ((bflags & 0x0800) !== 0) r.skip(24);
      if ((bflags & 0x2000) !== 0) r.skip(4);
      if ((bflags & 0x0020) !== 0) {
        r.skip(r.boneIndexSize);
        r.skip(4 + 4);
        const links = r.i32();
        for (let l = 0; l < links; l++) {
          r.skip(r.boneIndexSize);
          const limit = r.u8();
          if (limit === 1) r.skip(24);
        }
      }
    }
  }
  // morphs
  {
    const count = r.i32();
    for (let i = 0; i < count; i++) {
      r.text();
      r.text();
      r.skip(1);
      const type = r.u8();
      const offsets = r.i32();
      for (let o = 0; o < offsets; o++) {
        switch (type) {
          case 0: r.skip(r.morphIndexSize + 4); break;
          case 1: r.skip(r.vertexIndexSize + 12); break;
          case 2: r.skip(r.boneIndexSize + 12 + 16); break;
          case 3: case 4: case 5: case 6: case 7: r.skip(r.vertexIndexSize + 16); break;
          case 8: r.skip(r.materialIndexSize + 1 + 16 + 12 + 4 + 12 + 16 + 4 + 16 + 16 + 16); break;
          case 9: r.skip(r.morphIndexSize + 4); break;
          case 10: r.skip(r.rigidIndexSize + 1 + 12 + 12); break;
          default: throw new Error(`PMX: invalid morph type ${type}`);
        }
      }
    }
  }
  // display frames
  {
    const count = r.i32();
    for (let i = 0; i < count; i++) {
      r.text();
      r.text();
      r.skip(1);
      const elems = r.i32();
      for (let e = 0; e < elems; e++) {
        const target = r.u8();
        r.skip(target === 0 ? r.boneIndexSize : r.morphIndexSize);
      }
    }
  }
  // rigid bodies
  {
    const count = r.i32();
    for (let i = 0; i < count; i++) {
      const rb = new PmxRigidBody();
      rb.name = r.text();
      rb.nameEn = r.text();
      rb.boneIndex = r.index(r.boneIndexSize);
      rb.group = r.u8();
      rb.nonCollisionGroup = r.u16();
      rb.shapeType = r.u8();
      rb.size = r.vec3();
      rb.position = r.vec3();
      rb.rotation = r.vec3();
      rb.mass = r.f32();
      rb.linearDamping = r.f32();
      rb.angularDamping = r.f32();
      rb.restitution = r.f32();
      rb.friction = r.f32();
      rb.physicsMode = r.u8();
      model.rigidBodies.push(rb);
    }
  }
  // joints
  {
    const count = r.i32();
    for (let i = 0; i < count; i++) {
      const j = new PmxJoint();
      j.name = r.text();
      j.nameEn = r.text();
      j.jointType = r.u8();
      j.rigidBodyAIndex = r.index(r.rigidIndexSize);
      j.rigidBodyBIndex = r.index(r.rigidIndexSize);
      j.position = r.vec3();
      j.rotation = r.vec3();
      j.linearLowerLimit = r.vec3();
      j.linearUpperLimit = r.vec3();
      j.angularLowerLimit = r.vec3();
      j.angularUpperLimit = r.vec3();
      j.springLinear = r.vec3();
      j.springAngular = r.vec3();
      model.joints.push(j);
    }
  }
  // soft bodies (2.1)
  if (model.version >= F_2_1 && r.position < r.length) {
    const count = r.i32();
    for (let i = 0; i < count; i++) {
      const sb = new PmxSoftBody();
      sb.name = r.text();
      sb.nameEn = r.text();
      sb.shape = r.u8();
      sb.materialIndex = r.index(r.materialIndexSize);
      sb.group = r.u8();
      sb.nonCollisionGroup = r.u16();
      sb.flags = r.u8();
      sb.bLinkDistance = r.i32();
      sb.clusterCount = r.i32();
      sb.totalMass = r.f32();
      sb.collisionMargin = r.f32();
      sb.aeroModel = r.i32();
      for (let k = 0; k < 12; k++) sb.config.push(r.f32());
      for (let k = 0; k < 6; k++) sb.cluster.push(r.f32());
      for (let k = 0; k < 4; k++) sb.iteration.push(r.i32());
      for (let k = 0; k < 3; k++) sb.material.push(r.f32());
      const anchors = r.i32();
      for (let a = 0; a < anchors; a++) {
        const rigidBodyIndex = r.index(r.rigidIndexSize);
        const vertexIndex = r.vertexIndex();
        const nearMode = r.u8() !== 0;
        sb.anchors.push({ rigidBodyIndex, vertexIndex, nearMode });
      }
      const pins = r.i32();
      for (let p = 0; p < pins; p++) sb.pinVertices.push(r.vertexIndex());
      model.softBodies.push(sb);
    }
  }
  return model;
}
