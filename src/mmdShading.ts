import * as THREE from 'three';
import type { GLTF } from 'three/addons/loaders/GLTFLoader.js';
import { MMD_DEFAULT_TOONS } from './mmdDefaultToons';

// Reproduces MMD's own shading (as documented by MME's full.fx). MMD evaluates, in sRGB:
//
//   color = saturate(diffuse * lightColor + ambient) * texture   (+ or *) sphere
//   color = color * toon(N.L) + specular * lightColor * pow(N.H, power)
//
// three.js's MMDToonMaterial lights the model Phong-style instead, which differs in ways
// that are easy to see on MMD models:
//   - light is not clamped before multiplying the texture, so bright lights blow it out
//   - MMDLoader scales the PMX ambient color by 0.2 when the material has a texture
//   - only the red channel of the toon texture is used, losing tinted toon shadows
//
// The same shader is used for PMX/PMD (materials from MMDLoader) and for glTF files that
// carry the original PMX material data in extras.mmd (mmd2gltf / mmd2gltf-cs).

// MMD's default light color (154, 154, 154)
const MMD_LIGHT_COLOR = (154 / 255).toFixed(6);

const SPHERE_MULTIPLY = 1;
const SPHERE_ADD = 2;

interface MmdShadingParams {
  ambient: THREE.Color;
  sphere: THREE.Texture | null;
  sphereMode: number; // 0: none, 1: multiply (.sph), 2: add (.spa)
  toon: THREE.Texture | null;
  // MMDLoader rotates toon images to a horizontal ramp; original toon files are vertical
  toonLayout: 'horizontal' | 'vertical';
}

const PARS = /* glsl */ `
uniform vec3 mmdAmbient;
#ifdef MMD_SPHERE_MODE
	uniform sampler2D mmdSphere;
#endif
#ifdef MMD_TOON_LAYOUT
	uniform sampler2D mmdToon;
#endif
vec3 mmdToSRGB( vec3 c ) { return pow( max( c, 0.0 ), vec3( 1.0 / 2.2 ) ); }
vec3 mmdToLinear( vec3 c ) { return pow( max( c, 0.0 ), vec3( 2.2 ) ); }
`;

const CAPTURE_TEXEL = /* glsl */ `
vec4 mmdTexel = vec4( 1.0 );
#ifdef USE_MAP
	mmdTexel = texture2D( map, vMapUv );
#endif
`;

const SHADING = /* glsl */ `
vec3 mmdLightDir = vec3( 0.0, 0.0, 1.0 );
#if NUM_DIR_LIGHTS > 0
	mmdLightDir = directionalLights[ 0 ].direction;
#endif
vec3 mmdViewDir = normalize( vViewPosition );
float mmdDotNL = dot( normal, mmdLightDir );

vec3 mmdColor = clamp( mmdToSRGB( diffuse ) * ${MMD_LIGHT_COLOR} + mmdToSRGB( mmdAmbient ), 0.0, 1.0 );
mmdColor *= mmdToSRGB( mmdTexel.rgb );

#ifdef MMD_SPHERE_MODE
	// MMD maps the view-space normal directly (u = N.x * 0.5 + 0.5, v = 0.5 - N.y * 0.5),
	// with v = 0 at the top of the image. three.js's matcap formula assumes flipped
	// images and adds a perspective correction, which turned spheres upside down.
	vec2 mmdSphereUv = vec2( normal.x * 0.5 + 0.5, 0.5 - normal.y * 0.5 );
	vec3 mmdSphereColor = mmdToSRGB( texture2D( mmdSphere, mmdSphereUv ).rgb );
	#if MMD_SPHERE_MODE == ${SPHERE_MULTIPLY}
		mmdColor *= mmdSphereColor;
	#else
		mmdColor += mmdSphereColor;
	#endif
#endif

#ifdef MMD_TOON_LAYOUT
	#if MMD_TOON_LAYOUT == 1
		vec2 mmdToonUv = vec2( mmdDotNL * 0.5 + 0.5, 0.0 );
	#else
		vec2 mmdToonUv = vec2( 0.0, 0.5 - mmdDotNL * 0.5 );
	#endif
	mmdColor *= mmdToSRGB( texture2D( mmdToon, mmdToonUv ).rgb );
#endif

vec3 mmdHalf = normalize( mmdLightDir + mmdViewDir );
// pow( 0, 0 ) is undefined in GLSL and PMX specular power can be 0
mmdColor += pow( max( dot( mmdHalf, normal ), 0.0 ), max( shininess, 1e-4 ) ) * mmdToSRGB( specular ) * ${MMD_LIGHT_COLOR};

vec3 outgoingLight = mmdToLinear( clamp( mmdColor, 0.0, 1.0 ) );

// The sphere map is applied above; skip MMDToonShader's own matcap step (PMX materials)
#undef USE_MATCAP
`;

const OUTGOING_LIGHT =
  'vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + reflectedLight.directSpecular + reflectedLight.indirectSpecular + totalEmissiveRadiance;';

// MMD samples toon textures with LINEAR filtering, CLAMP addressing and no mipmaps, which
// turns the 2-color toon01-04 images into a narrow soft shadow edge. MMDLoader forces
// Nearest + Repeat instead (a jagged hard edge), and does so when the image finishes
// loading, so apply the settings after that.
function useMmdToonSampling(texture: THREE.Texture) {
  const apply = () => {
    texture.magFilter = THREE.LinearFilter;
    texture.minFilter = THREE.LinearFilter;
    texture.generateMipmaps = false;
    texture.wrapS = texture.wrapT = THREE.ClampToEdgeWrapping;
    texture.needsUpdate = true;
  };
  const pending = (texture as THREE.Texture & { readyCallbacks?: (() => void)[] }).readyCallbacks;
  if (pending) pending.push(apply);
  else apply();
}

// Works on MeshPhongMaterial and MMDLoader's MMDToonMaterial (both use the Phong shader)
function patchMaterial(material: THREE.Material, params: MmdShadingParams) {
  if (params.toon) useMmdToonSampling(params.toon);
  const sphereMode = params.sphere ? params.sphereMode : 0;
  const toonLayout = params.toon ? (params.toonLayout === 'horizontal' ? 1 : 2) : 0;

  material.onBeforeCompile = (shader) => {
    shader.uniforms.mmdAmbient = { value: params.ambient };
    shader.uniforms.mmdSphere = { value: params.sphere };
    shader.uniforms.mmdToon = { value: params.toon };
    const defines =
      (sphereMode ? `#define MMD_SPHERE_MODE ${sphereMode}\n` : '') +
      (toonLayout ? `#define MMD_TOON_LAYOUT ${toonLayout}\n` : '');
    shader.fragmentShader = defines + shader.fragmentShader
      .replace('#include <common>', '#include <common>\n' + PARS)
      .replace('#include <map_fragment>', '#include <map_fragment>\n' + CAPTURE_TEXEL)
      .replace(OUTGOING_LIGHT, SHADING);
  };
  material.customProgramCacheKey = () => `mmd-${sphereMode}-${toonLayout}`;
  material.needsUpdate = true;
}

// --- PMX / PMD (materials created by MMDLoader) ---

export function applyMmdShading(root: THREE.Object3D) {
  root.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (!mesh.isMesh) return;
    for (const material of Array.isArray(mesh.material) ? mesh.material : [mesh.material]) {
      if (material.type !== 'MMDToonMaterial') continue;
      const m = material as THREE.MeshPhongMaterial & {
        matcap: THREE.Texture | null;
        matcapCombine: THREE.Combine;
        gradientMap: THREE.Texture | null;
      };

      // Undo MMDLoader's 0.2 scale so the PMX ambient color is used as-is
      const ambient = m.emissive.clone();
      if (m.map) ambient.multiplyScalar(5);

      patchMaterial(m, {
        ambient,
        sphere: m.matcap,
        sphereMode: m.matcapCombine === THREE.MultiplyOperation ? SPHERE_MULTIPLY : SPHERE_ADD,
        toon: m.gradientMap,
        toonLayout: 'horizontal',
      });
    }
  });
}

// --- glTF with extras.mmd ---

export interface MmdExtrasMaterials {
  // Switches between the rebuilt MMD materials and the file's own glTF materials
  setEnabled: (enabled: boolean) => void;
  dispose: () => void;
}

const srgb = (rgb: number[]) => new THREE.Color().setRGB(rgb[0], rgb[1], rgb[2], THREE.SRGBColorSpace);

export function hasMmdExtras(gltf: GLTF) {
  return !!gltf.parser.json.extras?.mmd;
}

export async function buildMmdExtrasMaterials(gltf: GLTF): Promise<MmdExtrasMaterials | null> {
  const parser = gltf.parser;
  const json = parser.json;
  if (!json.extras?.mmd) return null;

  const createdTextures: THREE.Texture[] = [];
  const byIndex = new Map<number, Promise<THREE.Material | null>>();

  // Textures from the parser are shared with the glTF materials, so use clones with
  // the color space / wrapping MMD needs (clones share the image upload)
  const texture = async (index: number, setup?: (t: THREE.Texture) => void) => {
    const t = ((await parser.getDependency('texture', index)) as THREE.Texture).clone();
    t.colorSpace = THREE.SRGBColorSpace;
    setup?.(t);
    t.needsUpdate = true;
    createdTextures.push(t);
    return t;
  };
  const defaultToons = new Map<number, THREE.Texture>();
  const sharedToon = (shared: number) => {
    const index = shared + 1; // toonShared 0 = toon01.bmp
    if (!defaultToons.has(index)) {
      const t = new THREE.TextureLoader().load(MMD_DEFAULT_TOONS[index]);
      t.colorSpace = THREE.SRGBColorSpace;
      t.flipY = false;
      defaultToons.set(index, t);
      createdTextures.push(t);
    }
    return defaultToons.get(index)!;
  };

  const build = async (materialIndex: number): Promise<THREE.Material | null> => {
    const def = json.materials?.[materialIndex];
    const mmd = def?.extras?.mmd;
    if (!mmd) return null;

    const pbr = def.pbrMetallicRoughness ?? {};
    const factor: number[] = pbr.baseColorFactor ?? [1, 1, 1, 1];
    // origTexture is the unprocessed original image (alpha not pre-baked)
    const mapIndex = typeof mmd.origTexture === 'number' && mmd.origTexture >= 0 ? mmd.origTexture : pbr.baseColorTexture?.index;
    const map = mapIndex !== undefined ? await texture(mapIndex) : null;

    const sphereMode = mmd.sphereMode === SPHERE_MULTIPLY || mmd.sphereMode === SPHERE_ADD ? mmd.sphereMode : 0;
    const sphere = sphereMode && mmd.sphereTexture >= 0 ? await texture(mmd.sphereTexture) : null;

    let toon: THREE.Texture | null = null;
    if (mmd.toonTexture >= 0) toon = await texture(mmd.toonTexture);
    else if (mmd.toonShared >= 0 && mmd.toonShared <= 9) toon = sharedToon(mmd.toonShared);

    // For sphere-only materials (no texture, black diffuse/ambient) the converter stores the
    // sphere's average color in baseColorFactor so plain glTF viewers aren't black.
    // The PMX diffuse was ~0 then, which is what MMD shading needs.
    const ambient: number[] = mmd.ambient ?? [0, 0, 0];
    const sphereFallback = !map && sphere && Math.max(...ambient) < 0.05;
    const diffuse = sphereFallback ? [0, 0, 0] : factor;

    const material = new THREE.MeshPhongMaterial({
      name: def.name ?? '',
      map,
      color: srgb(diffuse),
      opacity: factor[3] ?? 1,
      specular: srgb(mmd.specular ?? [0, 0, 0]),
      shininess: mmd.specularPower ?? 0,
      side: (mmd.flags & 0x01) !== 0 || def.doubleSided ? THREE.DoubleSide : THREE.FrontSide,
    });

    // Unknown future alphaClass values are treated as "mask" (per the extras.mmd schema)
    const alphaClass = (mmd.alphaClass ?? (def.alphaMode ?? 'OPAQUE').toLowerCase()) as string;
    if (alphaClass === 'blend' || material.opacity < 1) material.transparent = true;
    else if (alphaClass !== 'opaque') material.alphaTest = def.alphaCutoff ?? 0.5;

    // Read by OutlineEffect; same mapping as MMDLoader
    const edge: number[] = mmd.edgeColor ?? [0, 0, 0, 1];
    material.userData.outlineParameters = {
      thickness: (mmd.edgeSize ?? 0) / 300,
      color: edge.slice(0, 3),
      alpha: edge[3],
      visible: (mmd.flags & 0x10) !== 0 && mmd.edgeSize > 0,
    };

    patchMaterial(material, { ambient: srgb(ambient), sphere, sphereMode, toon, toonLayout: 'vertical' });
    return material;
  };

  // Pair every mesh material with its MMD replacement
  const slots: { mesh: THREE.Mesh; slot: number; original: THREE.Material; mmd: THREE.Material }[] = [];
  const meshes: THREE.Mesh[] = [];
  gltf.scene.traverse((child) => {
    if ((child as THREE.Mesh).isMesh) meshes.push(child as THREE.Mesh);
  });

  for (const mesh of meshes) {
    const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
    for (let slot = 0; slot < materials.length; slot++) {
      const original = materials[slot];
      const index = parser.associations.get(original)?.materials;
      if (index === undefined) continue;
      if (!byIndex.has(index)) byIndex.set(index, build(index));
      const mmd = await byIndex.get(index)!;
      if (mmd) slots.push({ mesh, slot, original, mmd });
    }
  }

  const assign = (mesh: THREE.Mesh, slot: number, material: THREE.Material) => {
    if (Array.isArray(mesh.material)) mesh.material[slot] = material;
    else mesh.material = material;
  };

  return {
    setEnabled: (enabled) => {
      for (const s of slots) assign(s.mesh, s.slot, enabled ? s.mmd : s.original);
    },
    dispose: () => {
      // Whichever set is not on the meshes is not reached by the normal cleanup
      for (const s of slots) {
        const current = Array.isArray(s.mesh.material) ? s.mesh.material[s.slot] : s.mesh.material;
        (current === s.mmd ? s.original : s.mmd).dispose();
      }
      createdTextures.forEach((t) => t.dispose());
    },
  };
}
