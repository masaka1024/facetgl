# FacetGL

[日本語](README.md) | **English**

A 3D model viewer for Windows and macOS that opens glTF / GLB first and foremost, along with VRM, MMD (PMX/PMD), FBX, OBJ and STL.
Built with [Tauri](https://tauri.app/) and [three.js](https://threejs.org/).
It reproduces MMD's own shading for PMX files and for glTF files converted by [mmd2gltf](https://github.com/masaka1024/mmd2gltf-gui).

**🌐 Web version: <https://masaka1024.github.io/facetgl/>** — no installation needed. Just drop a model into your browser
(files are loaded only inside your browser and are never sent anywhere).

![Opening a .gltf that references external .bin / texture files, with the info panel shown](docs/screenshots/info-panel.png)

![Playing a GLB with animation](docs/screenshots/animation.png)

## Features

- **Drag & drop / "Open" / double-click** to open models (`.glb` `.gltf` `.vrm` can be associated with the app)
- **External files**: the `.bin` and textures of a `.gltf`, and the textures of PMX/FBX/OBJ files, are loaded automatically from the same folder.
  Textures missing from their referenced path are looked up by file name in the model's folder and its parent folders (up to 3 levels; nearer folders first, never above your home folder)
- **Animation playback**: play/pause, seek, speed control (glTF / FBX)
- **MMD motion (VMD)**: after opening a PMX / PMD, drop (or "Open") a VMD to play it. Supports IK and inherited (grant) parents, and can be combined with physics to make the model dance
- **Info panel**: file size; vertex, triangle, material, bone and morph counts; texture list; node hierarchy; rendering status;
  files that could not be loaded; VRM license terms
- **MMD rendering**: PMX/PMD are shaded with the same formulas as MMD (toon, sphere, edge)
- **Physics (experimental)**: hair and skirts of PMX models, and of glTF files converted by mmd2gltf, can sway with physics (`Alt` + drag moves the model).
  Computed by a custom Bullet 2.75–compatible engine tuned to match MMD's behavior
- **Supports mmd2gltf's `extras.mmd`**: converted GLB files can also be opened with MMD rendering, switchable with standard glTF rendering
- **View settings**: perspective / orthographic projection, field of view, outline width
- **Japanese / English UI**: follows your OS or browser language automatically (can be changed under "Language" in "View")
- Supports compressed glTF (Draco / KTX2 / Meshopt)

## Supported formats

| Format | Extensions | Notes |
|---|---|---|
| glTF 2.0 | `.glb` `.gltf` | Draco / KTX2 / Meshopt, `extras.mmd` |
| VRM | `.vrm` | 0.x / 1.0 |
| MMD | `.pmx` `.pmd` | Motion `.vmd` (load after opening the model). Physics for PMX only (for glTF, files converted by mmd2gltf) |
| FBX | `.fbx` | Binary / ASCII (7.0 or later) |
| OBJ | `.obj` | Loads `.mtl` and textures from the same folder |
| STL | `.stl` | Displayed with Z axis up |

## Web version

Available as-is at <https://masaka1024.github.io/facetgl/>, with the same rendering and features as the desktop version.

- For models whose textures are separate files (such as PMX / FBX / OBJ), and for `.gltf` files that reference external files,
  **drop the whole folder** (or use "Open folder"). GLB / VRM can be opened as a single file
- **Physics and MMD motion (VMD) also work in the web version**. After opening a PMX together with its folder,
  drop (or "Open") a single VMD file to play it on that model
- Physics speed depends on your browser and PC. Current Chrome / Edge / Firefox / Safari (15 or later)
  compute it quickly with WebAssembly (where that is unavailable, it still runs with identical results, just a little slower)
- Due to how browsers work, you need to drop (or select) the files each time you open them
- Desktop-only features: file association (open with a double-click), and reading files directly by path from your PC or NAS

## Installation (desktop version)

Download `FacetGL_x.y.z_x64-setup.exe` from [Releases](https://github.com/masaka1024/facetgl/releases) and run it.

### Windows

Because the app is not code-signed, Windows may show "Windows protected your PC" the first time.
Click "More info" → "Run anyway" to continue.

### macOS (11 Big Sur or later, Apple Silicon / Intel)

1. Download `FacetGL_x.y.z_universal.dmg` from [Releases](https://github.com/masaka1024/facetgl/releases) and open it
2. Drag `FacetGL` into the "Applications" folder
3. Because the app is not notarized by Apple, macOS will say it cannot be opened the first time.
   Close the dialog, then open **System Settings → Privacy & Security**,
   and click **"Open Anyway"** next to "FacetGL" near the bottom (from the second launch on, it opens normally)

If that does not work, run the following in Terminal and then open the app.

```sh
xattr -dr com.apple.quarantine /Applications/FacetGL.app
```

## Controls

| Input | Action |
|---|---|
| Left drag / right drag / wheel | Rotate / pan / zoom |
| `Ctrl+O` (`⌘O` on Mac) | Open a file (a model, or a motion for the open MMD model) |
| `Space` | Play / pause animation |
| `I` | Show / hide the info panel |
| `P` | Toggle perspective / orthographic projection |
| `F` | Physics ON / OFF (PMX / mmd2gltf glTF, experimental) |
| `R` | Reset physics (return to the initial pose and start over) |
| `Alt` + drag (`option` + drag on Mac) | With physics ON, move the model to make hair and skirts sway (it returns to its original position when released) |

## About MMD rendering

PMX/PMD files, and glTF files with `extras.mmd` produced by mmd2gltf, are rendered with MMD's shading reproduced.

![A GLB converted by mmd2gltf opened with MMD rendering, showing the extras.mmd information (Niconi Solid-chan / Alicia Solid)](docs/screenshots/mmd-extras.png)

```
color = saturate(diffuse × light color + ambient) × texture (+ / × sphere)
color = color × toon(N·L) + specular × light color × pow(N·H, specular power)
```

- Computed in sRGB space just like MMD, with MMD's default light color (154, 154, 154)
- Toon textures are sampled with LINEAR filtering, CLAMP and no mipmaps (same as MMD)
- Sphere maps are sampled using the view-space normal (`u = N.x·0.5+0.5`, `v = 0.5−N.y·0.5`)
- Outlines are "edge size × setting" pixels (default 1.0 px). Per-vertex edge scale (`_EDGESCALE`) is not supported
- Self-shadows are not drawn, matching MMD's default settings

## Physics (experimental)

When a PMX has rigid bodies, a "Physics: OFF" button appears; pressing it makes hair, skirts and other rigid bodies move with physics.
When a motion (VMD) is loaded, they sway along with the dance. glTF / GLB files converted by mmd2gltf can also run physics in the same way,
using the rigid bodies and joints stored in `extras.mmd`, and sway along with the animation contained in the file. When there is no motion,
`Alt` (`option` on Mac) + drag lets you move the model to check the sway.

- Physics is computed by a TypeScript port (`src/physics/engine/`) of
  [mmd2gltf-cs-physics](https://github.com/masaka1024/mmd2gltf-cs-physics), a custom engine that reproduces MMD (PMX Editor) physics
  in a Bullet 2.75–compatible way. It is the same engine used by the Unity / UE5 importers
- The port performs all calculations with the same rounding as 32-bit floats (C# `float`).
  With the same model and the same input, the position, rotation and velocity of every rigid body have been confirmed to match the C# version bit for bit
- Targets are PMX and the `extras.mmd` of glTF files converted by mmd2gltf (PMD is not supported). glTF is assembled from the same rigid bodies,
  joints and bone positions as the PMX, so the same input gives the same result as the PMX
- When physics results are already baked into a glTF animation, the baked animation takes priority.
  This is decided per bone: rigid bodies on bones driven by the animation follow the animation,
  and in files where only the body is baked, hair and skirts are swayed by physics. If all physics bones are baked,
  the button shows "Physics: Baked" and cannot be pressed
- The "(4.0 ms)" on the button is the physics time per frame. Models with more rigid bodies are heavier
- While physics is ON, the motion advances by the same amount as physics. Even if frames drop on a slow PC, bones will not jump suddenly
  and "kick" the hair (at the cost of slightly slower playback). When the motion loops or you seek,
  rigid bodies are re-placed around the new pose

## Building from source

Requirements: [Node.js](https://nodejs.org/) 18 or later, [Rust](https://rustup.rs/), and

- Windows: Visual Studio Build Tools (C++), WebView2 (built into Windows 10/11)
- macOS: Xcode Command Line Tools (`xcode-select --install`)

```sh
npm install
npm run tauri dev # launch for development
npm run tauri build # create the installer
# Windows: src-tauri/target/release/bundle/nsis/
# macOS: src-tauri/target/release/bundle/dmg/ (and macos/FacetGL.app)
```

To build a Mac version (Universal) that runs on both Apple Silicon and Intel:

```sh
rustup target add aarch64-apple-darwin x86_64-apple-darwin
npm run tauri build -- --target universal-apple-darwin
```

Running "Build desktop apps" from the GitHub Actions tab builds both the Windows `.exe` and the Mac `.dmg` together.
Pushing a tag such as `v0.3.0` automatically attaches both to the release for that version.

### Physics engine parity test

The physics engine (`src/physics/engine/`) is ported to produce bit-identical results with the C# version,
[mmd2gltf-cs-physics](https://github.com/masaka1024/mmd2gltf-cs-physics). Whenever you change either one,
check that they still match with `npm run parity -- --model <model.pmx> --engine <path to mmd2gltf-cs-physics>`
(requires .NET SDK 8 or later; see [tools/physics-parity/README.md](tools/physics-parity/README.md) for details).

## License

[MIT License](LICENSE). For bundled third-party software, see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

For the terms of use of the models you display, follow the terms of each model's distributor.

### Models in the screenshots

Models from [Khronos glTF Sample Assets](https://github.com/KhronosGroup/glTF-Sample-Assets) are used.

- **FlightHelmet** — © 2018, Public. [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/)
- **Fox** — © 2014, tomkranis; glTF conversion © 2017, @AsoboStudio and @scurest.
  [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)

The MMD rendering screenshot (`docs/screenshots/mmd-extras.png`) uses
[Niconi Solid-chan (Alicia Solid)](https://3d.nicovideo.jp/alicia/) (© dwango, inc.) under its
[terms of use](https://3d.nicovideo.jp/alicia/rule.html).
This image is not covered by the MIT License. FacetGL is not an official product of DWANGO Co., Ltd.
