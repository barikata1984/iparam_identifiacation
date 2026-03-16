# Log: TLS スケーリング行列分析

## 2026-03-12: 理論分析

Kubus et al. (2008) の RTLS 論文と Golub & Van Loan (2012) §6.3 を精読し、
現在の TLS 実装における重み行列 D, T の構成方法を理論と比較した。

### 理論的背景

#### Golub & Van Loan の定式化 (Eq. 6.3.3)

Weighted TLS は `min ||D[E|R]T||_F` を解く。

- **T = diag(t_1, ..., t_{n+1})**: 列（変数）の重み。大きい t_i → 列 i の誤差をより強く罰する → 列 i をより信頼する
- **D = diag(d_1, ..., d_m)**: 行（観測）の重み。大きい d_i → 観測 i をより信頼する

§6.3.3 の幾何学的解釈 (Eq. 6.3.6) から、T は各変数のノイズレベルの逆数 `t_i = 1/sigma_noise_i` として設定するのが理論的に正しい。

#### Kubus の RTLS (Eq. 26)

`N_k = [D [A_k (f_k; tau_k)] T]^T` — Golub & Van Loan の `C = D[A|b]T` に直接対応。
論文では D, T の具体的な値の決め方は記載なし。

### 現在の実装との乖離

バッチ TLS (`tls.py`) の 3 モード:
- `NONE`: D=I, T=I
- `COLUMN_ONLY` (デフォルト): T = diag(1/std(各列)), D = I
- `FULL`: T = diag(1/std(各列)), D = diag(1/std(各行))

**問題**: std はデータのばらつき（信号+ノイズ）であり、ノイズレベルではない。
大きく変動する列（良い加振 = 高 SNR）が小さい t を持ち、「信頼度が低い」と扱われてしまう。

再帰的 TLS (`recursive_tls.py`) では D, T スケーリングは未実装。

### 改善案

`ScalingMode.NOISE_BASED` を追加:
- diff ベースのノイズ推定: `noise_std = std(diff(column)) / sqrt(2)`
- 500Hz サンプリングなら信号の滑らかな変動は diff で消え、ノイズが残る
- `t_i = 1 / noise_std_i`

### 参考文献

- Kubus, D., Kröger, T., & Wahl, F. M. (2008). On-line estimation of inertial parameters using a recursive total least-squares approach. IROS.
- Golub, G. H., & Van Loan, C. F. (2012). Matrix Computations, 4th ed., §6.3.
