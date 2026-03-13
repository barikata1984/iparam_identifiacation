# TLS スケーリング行列の理論分析

> **日付**: 2026-03-12

## 概要

Kubus et al. (2008) の RTLS 論文と Golub & Van Loan (2012) §6.3 を精読し、
現在の TLS 実装における重み行列 D, T の構成方法を理論と比較した。

## 理論的背景

### Golub & Van Loan の定式化 (Eq. 6.3.3)

Weighted TLS は `min ||D[E|R]T||_F` を解く。

- **T = diag(t₁, ..., tₙ₊₁)**: 列（変数）の重み。大きい t_i → 列 i の誤差をより強く罰する → 列 i をより信頼する
- **D = diag(d₁, ..., dₘ)**: 行（観測）の重み。大きい d_i → 観測 i をより信頼する

§6.3.3 の幾何学的解釈 (Eq. 6.3.6) から、T は R^(n+1) における距離計量を定義し、
各変数のノイズレベルの逆数 `t_i = 1/σ_noise_i` として設定するのが理論的に正しい。

### Kubus の RTLS (Eq. 26)

`N_k = [D [A_k (f_k; τ_k)] T]^T` — Golub & Van Loan の `C = D[A|b]T` に直接対応。
論文では D, T の具体的な値の決め方は記載なし。

## 現在の実装との乖離

### バッチ TLS (`tls.py`)

3つの ScalingMode を実装済み:
- `NONE`: D=I, T=I
- `COLUMN_ONLY` (デフォルト): T = diag(1/std(各列)), D = I
- `FULL`: T = diag(1/std(各列)), D = diag(1/std(各行))

**問題点**: std はデータのばらつき（信号+ノイズ）であり、ノイズレベルではない。
大きく変動する列（良い加振 = 高 SNR）が小さい t を持ち、「信頼度が低い」と扱われてしまう。

### 再帰的 TLS (`recursive_tls.py`)

D, T に相当するスケーリングは**未実装**。

## 改善案

### T（列の重み）: ノイズ推定ベース

新しい `ScalingMode.NOISE_BASED` を追加する。

1. **diff ベースのノイズ推定**（データ駆動、センサ仕様不要）:
   - `noise_std = std(diff(column)) / √2` — 隣接サンプル差分からノイズ成分を抽出
   - 500Hz サンプリングなら信号の滑らかな変動は diff で消え、ノイズが残る
   - `t_i = 1 / noise_std_i`

2. **センサ仕様ベース**（より精緻、要パラメータ設定）:
   - UR5e F/T センサと数値微分のノイズ仕様から設定
   - リグレッサ列（運動学ノイズ大）と レンチ列（F/T ノイズ小）を区別

### D（行の重み）

- 最小限: D = I のまま（Kubus も具体的な値を示していない）
- 改善案: 各行のリグレッサノルムで重み付け（低加振サンプルのダウンウェイト）

## 次のステップ

1. `ScalingMode.NOISE_BASED` を `tls.py` に実装（diff ベースの T + D=I）
2. 既存の3モードと並列比較実験（OLS との乖離が改善されるか検証）
3. 効果確認後、再帰的 TLS にもスケーリングを組み込む

## 参考文献

- Kubus, D., Kröger, T., & Wahl, F. M. (2008). On-line estimation of inertial parameters using a recursive total least-squares approach. IROS.
- Golub, G. H., & Van Loan, C. F. (2012). Matrix Computations, 4th ed., §6.3.
