# FacetGL

glTF / GLB を中心に、VRM・MMD (PMX/PMD)・FBX・OBJ・STL を開ける Windows 向けの 3D モデルビューアです。
[Tauri](https://tauri.app/) と [three.js](https://threejs.org/) で作られています。

*A lightweight desktop 3D model viewer for glTF / GLB, VRM, MMD (PMX/PMD), FBX, OBJ and STL,
built with Tauri and three.js. It reproduces MMD's own shading for PMX files and for glTF
files converted by [mmd2gltf](https://github.com/masaka1024/mmd2gltf-gui).*

## 特徴

- **ドラッグ＆ドロップ / 「開く」 / ダブルクリック** でモデルを開く (`.glb` `.gltf` `.vrm` を関連付け)
- **外部ファイル対応**: `.gltf` の `.bin` やテクスチャ、PMX/FBX/OBJ のテクスチャを同じフォルダから自動で読む。
  参照先に無いテクスチャは、フォルダ内 (3階層まで) の同名ファイルで補う
- **アニメーション再生**: 再生/一時停止、シーク、速度変更 (glTF / FBX)
- **情報パネル**: ファイルサイズ、頂点・三角形・材質・ボーン・モーフ数、テクスチャ一覧、ノード階層、描画状況、
  読み込めなかったファイル、VRM の利用条件
- **MMD 表示**: PMX/PMD を MMD と同じ式でシェーディング (トゥーン・スフィア・エッジ)
- **mmd2gltf の `extras.mmd` に対応**: 変換済み GLB も MMD 表示で開け、glTF 標準表示と切り替え可能
- **表示設定**: 透視 / 平行投影、視野角、輪郭線の太さ
- 圧縮 glTF (Draco / KTX2 / Meshopt) に対応

## 対応形式

| 形式 | 拡張子 | 備考 |
|---|---|---|
| glTF 2.0 | `.glb` `.gltf` | Draco / KTX2 / Meshopt、`extras.mmd` |
| VRM | `.vrm` | 0.x / 1.0 |
| MMD | `.pmx` `.pmd` | モーション (VMD) は未対応 |
| FBX | `.fbx` | バイナリ / ASCII (7.0 以降) |
| OBJ | `.obj` | `.mtl` とテクスチャを同じフォルダから読み込み |
| STL | `.stl` | Z 軸が上の向きとして表示 |

## インストール

[Releases](https://github.com/masaka1024/facetgl/releases) から `FacetGL_x.y.z_x64-setup.exe` をダウンロードして実行してください。

電子署名をしていないため、初回は「Windows によって PC が保護されました」と表示されることがあります。
「詳細情報」→「実行」で進めてください。

## 操作

| 操作 | 内容 |
|---|---|
| 左ドラッグ / 右ドラッグ / ホイール | 回転 / 平行移動 / ズーム |
| `Ctrl+O` | ファイルを開く |
| `Space` | アニメーションの再生 / 一時停止 |
| `I` | 情報パネルの表示 / 非表示 |
| `P` | 透視 / 平行投影の切り替え |

## MMD 表示について

PMX/PMD と、mmd2gltf が出力する `extras.mmd` 付きの glTF は、MMD のシェーディングを再現して描画します。

```
color = saturate(拡散色 × ライト色 + 環境色) × テクスチャ  (+ / × スフィア)
color = color × トゥーン(N·L) + スペキュラ × ライト色 × pow(N·H, 反射強度)
```

- 計算は MMD と同じく sRGB 空間で行い、ライト色は MMD の初期値 (154, 154, 154)
- トゥーンは LINEAR・CLAMP・ミップマップなしで参照 (MMD と同じ)
- スフィアは view 空間の法線で参照 (`u = N.x·0.5+0.5`, `v = 0.5−N.y·0.5`)
- 輪郭線は「エッジサイズ × 設定値」ピクセル (初期値 1.0 px)。頂点ごとのエッジ倍率 (`_EDGESCALE`) は未対応
- セルフシャドウは MMD の初期設定に合わせて描画しない

## ソースからビルド

必要なもの: [Node.js](https://nodejs.org/) 18 以降、[Rust](https://rustup.rs/)、
Visual Studio Build Tools (C++)、WebView2 (Windows 10/11 は標準搭載)

```sh
npm install
npm run tauri dev     # 開発用に起動
npm run tauri build   # インストーラーを作成 (src-tauri/target/release/bundle/nsis/)
```

## ライセンス

[MIT License](LICENSE)。同梱しているサードパーティのソフトウェアについては
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) を参照してください。

表示するモデルの利用条件は、各モデルの配布元の規約に従ってください。
