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

## 2026-03-25: 文献調査と実装

### 文献調査

WTLS の重み行列選択について 26 論文を調査（`docs/SURVEYS/wtls_scaling_matrix.md`）。
主要な知見:

1. **WTLS = ML（ガウスノイズ下）**: W = Σ^{-1}（ノイズ共分散の逆行列）が統計的に最適。
   Kukush & Van Huffel (2004), Markovsky & Van Huffel (2007), Crassidis & Cheng (2019) で独立に確認。
2. **Golub & Van Loan, Kubus ともに D, T の選び方を規定していない**: 応用側の判断に委ねられている。
3. **列スケール正規化 ≠ ノイズベース重み付け**: 前者は数値的条件改善、後者は統計的最適性。目的が異なる。
4. **2026-03-12 のログの訂正**: 「§6.3.3 から t_i = 1/σ_noise_i が理論的に正しい」は
   Golub & Van Loan の直接的記述ではなく、ガウスノイズ仮定下の ML 解釈からの帰結。

### ScalingMode 再設計

旧モード → 新モードへの変更:

| 旧 | 新 | 理由 |
|---|---|---|
| `NONE` | `IDENTITY` | 明示的に W=I を示す |
| `COLUMN_ONLY` | `DATA_VARIANCE` | 「何をスケールするか」ではなく「何に基づくか」で命名 |
| `FULL` | (削除) | 行の std 正規化に理論的根拠なし |
| (なし) | `NOISE_VARIANCE` | ML 最適。`σ = std(diff(col))/√2` |

デフォルトを `NOISE_VARIANCE` に変更。テスト 12/12 パス。

### diff ベースノイズ推定の根拠

500Hz サンプリングでは隣接サンプル間の信号変化は滑らかなので、
`diff(col)` は信号成分を除去しノイズ成分を分離する。
`std(diff) / √2` は i.i.d. ノイズの標準偏差の不偏推定量。
測地学の LS-VCE (Amiri-Simkooei 2013) の簡易版として正当化可能。

### Partial EIV 実装と実機検証

TLS+bias でバイアス列（定数 = エラーフリー）を通常 TLS と同様に摂動させていたため、
質量が系統的に過大推定されていた（1.60 kg vs 真値 0.95 kg）。

Van Huffel & Vandewalle (1989) の Generalized TLS に基づき Partial EIV を実装:
- バイアス列を `error_free_cols` に指定
- 射影法で A1（エラーフリー列）の影響を除去 → 縮小 TLS → OLS で復元
- 合成データテスト 3 件追加（Full TLS より Partial EIV が高精度を確認）

実機 5 試行（グリッパのみ、真値 ≈ 0.95 kg）:

| 手法 | 質量平均 [kg] | 誤差率 | CV% |
|---|---|---|---|
| OLS | 0.041 | -95.7% | 33.6% |
| TLS | 0.051 | -94.6% | 34.1% |
| OLS+bias | **1.010** | **+6.3%** | **0.65%** |
| TLS+bias (Partial EIV) | 1.357 | +42.8% | 0.72% |
| TLS+bias (Full TLS, 改修前) | 1.603 | +68.7% | 0.29% |

Partial EIV で TLS+bias は 1.60→1.36 kg に改善したが、+43% の過大推定が残存。
原因: リグレッサ内の重力列 (g_x, g_y, g_z) がバイアス列と部分的に相関しており、
射影で完全に分離できない。

**結論**: OLS+bias (m≈1.01 kg, +6%) が現時点で最良の推定手法。
TLS+bias のさらなる改善は励起軌道の条件数改善か Structured TLS が必要だが、優先度は低い。
