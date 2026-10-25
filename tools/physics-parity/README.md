# 物理エンジン 一致テスト (physics-parity)

FacetGL の物理エンジン (`src/physics/engine/`) は、C# のエンジン
[mmd2gltf-cs-physics](https://github.com/masaka1024/mmd2gltf-cs-physics) を TypeScript へ移植したものです。
計算をすべて C# の `float` と同じ丸めで行っているので、**同じ入力なら C# 版とビット単位で同じ結果**になります。

このフォルダの道具は、それが今も成り立っているかを確かめるためのものです。
次のようなときに実行してください。

- C# エンジンを直したあと (TypeScript 版にも同じ変更を入れたら、一致するかを確認)
- TypeScript 版を高速化・整理したあと (結果が1ビットも変わっていないかを確認)

## 必要なもの

- Node.js (FacetGL のビルドと同じもの。`npm install` 済みであること)
- [.NET SDK](https://dotnet.microsoft.com/download) 8 以降 (`dotnet --version` で確認)
- C# エンジンのソース: `mmd2gltf-cs-physics` を clone したフォルダ
- 剛体のある PMX モデル 1 体 (**リポジトリには入れないこと**。`test-models/` に置けば Git の対象外です)

## 使い方

```sh
npm run parity -- --model <モデル.pmx> --engine <mmd2gltf-cs-physics のフォルダ>
```

- `--engine` を省くと、FacetGL と同じ階層にある `mmd2gltf-cs-physics` フォルダを使います
- `--quick` を付けると既定設定の 1 通りだけを確認します (数十秒)

例 (Windows):

```sh
npm run parity -- --model test-models\IA.pmx --engine C:\src\mmd2gltf-cs-physics
```

## やっていること

1. C# エンジンのソースからダンプ用プログラム (`dumper/`) をビルドする
2. C# 版と TypeScript 版 (`runner.ts`) で、同じシナリオを同じ設定で動かす
   - 静止 60 フレーム → 全身を揺さぶる 180 フレーム → 静止 180 フレーム (1/60 秒刻み・2 サブステップ)
   - 設定は Unity インポーター・FacetGL と同じ (重力 98、反復 10)
3. 毎フレーム、全剛体の位置・回転・速度を比べる (`compare.mjs`)

確認するのは次の 7 通りです。

| 確認 | 中身 |
|---|---|
| 既定設定 | 普段使う設定 |
| 既定設定 (TypeScript のみ) | 高速化用の WebAssembly 部品を使わない経路 |
| 既定外の設定 1・2 | ふだん使わない切り替え (split impulse など) の経路 |
| 1ulp ずらした開始 × 2 | 開始状態をわずかに変えても、両者が同じように動くか |

## 結果の見方

```
✓ 既定設定                         420 フレームすべてビット一致
...
参考: C# 自身で1ulpずらすと、最終フレームで 407 剛体が最大 3.40e-1 ずれる (系の敏感さの目安)
剛体 606 / 1/60 秒あたり: C# 4.33 ms, TypeScript 5.01 ms
=== すべて一致 ===
```

- **すべて ✓** なら OK です
- **✗ (フレーム N から不一致)** なら、どこかに違いがあります。
  「フレーム 0 から」なら計算式や設定の違い、後半から少しずつなら丸めの順序の違いが疑わしいです
- 「参考」の行は、C# 版自身で開始状態を 1ulp (float の最小の差) だけずらしたときのずれ方です。
  揺れ物は小さな差がすぐ大きく育つので、**完全一致はとても強い確認**になります
- 最後に、エンジン付属テスト (`tools/compilecheck`) の TypeScript 版の結果も参考として表示します。
  モデルによっては C# 版でも同じ項目が FAIL するので、C# 版の結果と見比べてください

## ファイル

| ファイル | 役割 |
|---|---|
| `check.mjs` | 全体をまとめて実行する (`npm run parity`) |
| `dumper/` | C# 版のダンプ用プログラム (エンジンのソースをそのままビルドして使う) |
| `runner.ts` | TypeScript 版のダンプ用プログラム |
| `compare.mjs` | 2 つのダンプをビット単位で比べる (単独でも使える: `node compare.mjs a.bin b.bin`) |
| `harness.ts` | エンジン付属テストの TypeScript 版 |

途中のファイルは `.out/` に作られます (Git の対象外)。
