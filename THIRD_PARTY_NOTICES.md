# Third-party notices

FacetGL is distributed under the MIT License (see `LICENSE`). It includes or bundles the
following third-party software.

| Component | Version | License | Where it is used |
|---|---|---|---|
| [three.js](https://github.com/mrdoob/three.js) | 0.160 | MIT | Rendering, loaders (glTF, MMD, FBX, OBJ/MTL, STL, TGA), OutlineEffect, meshopt decoder, fflate, mmd-parser |
| [@pixiv/three-vrm](https://github.com/pixiv/three-vrm) | 3.5 | MIT | VRM loading (MToon materials, spring bones, expressions) |
| [Tauri](https://github.com/tauri-apps/tauri) (`tauri`, `@tauri-apps/api`, `@tauri-apps/plugin-dialog`) | 2.x | MIT OR Apache-2.0 | Desktop application shell |
| [Draco](https://github.com/google/draco) decoder | from three.js 0.160 | Apache-2.0 | `public/libs/draco/` (Draco-compressed glTF) |
| [Basis Universal](https://github.com/BinomialLLC/basis_universal) transcoder | from three.js 0.160 | Apache-2.0 | `public/libs/basis/` (KTX2 textures) |

## MMD physics engine

`src/physics/engine/` is a TypeScript port of
[mmd2gltf-cs-physics](https://github.com/masaka1024/mmd2gltf-cs-physics)
(MIT License, Copyright (c) 2026 masaka1024), the same author's Bullet 2.75-compatible
MMD physics engine.

## MMD shared toon textures

`src/mmdDefaultToons.ts` contains the shared toon textures `toon00.bmp`–`toon10.bmp` as
embedded in three.js r160 `examples/jsm/loaders/MMDLoader.js` (MIT License,
Copyright 2010-2023 Three.js Authors).

## Rust crates

The Rust side depends on crates from crates.io (listed in `src-tauri/Cargo.lock`), which
are mostly MIT and/or Apache-2.0 licensed. `cargo about` or `cargo license` can produce a
full list.
