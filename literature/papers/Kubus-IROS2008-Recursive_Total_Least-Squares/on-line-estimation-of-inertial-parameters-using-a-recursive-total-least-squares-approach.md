---
Title: "On-Line Estimation of Inertial Parameters Using a Recursive Total Least-Squares Approach"
Authors:
  - Kubus, Daniel
  - Kroeger, Torsten
  - Wahl, Friedrich M.
Year: 2008
Venue: IROS
Tags:
  - "inertial-parameter-estimation"
  - "total-least-squares"
  - "recursive-estimation"
  - "force-torque-sensor"
  - "incremental-svd"
PDF: "[[papers/Kubus-IROS2008-Recursive_Total_Least-Squares/main.pdf|📃]]"
Import Date: "2026-06-08"
Read Date: 2026-06-08
Executive Summary: "マニピュレータ負荷の 10 慣性パラメータをオンライン推定するための再帰的 Total Least-Squares (RTLS) 手法を提案. 従来の RLS や RIV が観測行列 (リグレッサ) の誤差を無視するのに対し, RTLS はデータ行列と計測ベクトルの双方の誤差を考慮する EIV モデル (eq. 23) に基づく. Brand のインクリメンタル SVD [18] を活用し, 約 1.5 秒でオンライン推定を完了. RLS・RIV との比較実験で, 使用する加速度信号源・軌道・負荷によらず RTLS が最良の推定精度を示した."
Citekey: Kubus-IROS2008-Recursive_Total_Least-Squares
BibTeX Key: kubus2008online
DOI: "10.1109/IROS.2008.4650772"
Relevance: 5
Repository: "none"
Category: note
Template Version: v2.3
---

## Executive Summary
マニピュレータ負荷の 10 慣性パラメータをオンライン推定するための再帰的 Total Least-Squares (RTLS) 手法を提案. 従来の RLS や RIV が観測行列 (リグレッサ) の誤差を無視するのに対し, RTLS はデータ行列と計測ベクトルの双方の誤差を考慮する EIV モデル (eq. 23) に基づく. Brand のインクリメンタル SVD [18] を活用し, 約 1.5 秒でオンライン推定を完了. RLS・RIV との比較実験で, 使用する加速度信号源・軌道・負荷によらず RTLS が最良の推定精度を示した.

---
## Summary

### この論文が答えた問い, あるいは解決した課題は何か？

慣性パラメータのオンライン推定において, 従来の最小二乗系手法 (RLS, RIV) がデータ行列 (リグレッサ) 中の加速度・角速度信号のノイズと外乱を無視している問題を解決すること. (§I) 特に産業用マニピュレータでは関節角セットポイントと実際の動作の乖離, 加速度センサのノイズ特性 [10] により, データ行列の誤差は無視できない.

### 提案手法のアプローチと, その根幹をなす要素は何か？

Newton-Euler に基づくリグレッサ行列と計測レンチの**双方に誤差を仮定する** Errors-in-Variables (EIV) モデルを採用し, TLS 問題をインクリメンタル SVD で再帰的に解く.

- **EIV 誤差モデル** (eq. 23): `[f; τ] + e = (A_Ξ + E)·φ`. データ行列 A の誤差 E とレンチの誤差 e を同時に考慮. OLS/RLS の `[f; τ] + e = A·φ` (eq. 16) と対比される.
- **インクリメンタル SVD** (eq. 24–36, Brand [18] に基づく): 新しいデータ行列・レンチベクトルが到着するたびに, 既存の SVD を O(r²) で更新. 右特異ベクトルの更新を省略し計算コストを削減. 最小特異値に対応する左特異ベクトルから TLS 解を抽出 (eq. 36).
- **対角重み行列 D と T** (eq. 24, 26): 行 (時刻) 方向の重み D と列 (パラメータ) 方向の重み T による前処理. 2007 論文のリグレッサ構造・センサオフセット補償 (eq. 1–9) はそのまま踏襲.
- **センサオフセット補償**: 2007 論文と同一の 2 方式 (零点補正 + 補正行列 / オフセット直接推定, eq. 7–9). 本論文は推定アルゴリズムの改善に焦点を当て, オフセット処理自体は先行研究 [4] を参照.

### 特に参考とした既存研究と, それらと比した提案手法の新規性は何か？

- Kubus et al. 2007 [4] (= [[papers/Kubus-IROS2007-Rigid_Object_Recognition/on-line-rigid-object-recognition-and-pose-estimation-based-on-inertial-parameters|Kubus+ 2007]]): RIV 法による推定. データ行列誤差を操作変数で間接的に緩和するが, 行列自体の誤差は明示的に扱わない.
- An et al. [5], Kozlowski [6], Niebergall [7], Winkler [8]: いずれも OLS ベースのオフライン推定. センサオフセットやデータ行列誤差を考慮せず.
- Brand [18]: 不完全データに対するインクリメンタル SVD. RTLS のオンライン化を可能にする計算基盤.
- Golub & Van Loan [22]: バッチ TLS の標準的定式化.

新規性: (1) 慣性パラメータ推定に TLS を初めて適用し, データ行列の誤差を明示的に扱った, (2) Brand のインクリメンタル SVD を組み込み, バッチ SVD なしでのオンライン TLS 推定を実現, (3) RLS・RIV との体系的な比較実験で, 信号源・軌道・負荷に依存しない RTLS の優位性を実証.

### どのように訓練・最適化したのか？

- **損失関数 / 最適化目的**: N/A: 学習ベースではない. TLS は `||[E | e]||_F` (Frobenius ノルム) を最小化 (eq. 23). 励起軌道の条件数 κ(Υ) を Monte Carlo + fmincon で最小化 (§III, 2007 論文 [4] と同一手法).
- **データセット**: 実験データ. Staubli RX60, JR3 85M35A3-I40-D 200N12 F/T・加速度センサ, ADXRS300 角速度センサ. テスト負荷 3 個 (1.0–1.6 kg の鉄ブロック, Fig. 5), 1 kHz サンプリング, 推定時間 1.5 秒. 2 軌道 (tr1, tr2) × 2 負荷 (o1, o2) で評価. 遠位センサ部の慣性パラメータは [4] の結果を差し引いて補正.

### どのように検証したか？指標と結果は？

相対推定誤差 e_rel (eq. 37) で RLS, RIV, RTLS を比較.

- **加速度センサ信号使用時** (Fig. 6, o1, tr2): 全 10 パラメータで RTLS が最良. 例えば m の相対誤差: RLS 約 8%, RIV 約 4%, RTLS 約 1%. 慣性テンソル要素 (Ixx 等) では RLS が 20–35% の誤差に対し RTLS は 5% 以下.
- **KF フィルタ信号使用時** (Fig. 7, o1, tr2): 全体的に誤差が低下するが, 依然 RTLS が最良. RTLS の m 誤差は 1% 未満.
- **信号源の影響** (Fig. 9): JR3 加速度 + KF 角速度の混合信号 (Mixed) が RTLS では最良の結果. 純粋な KF 信号は RLS/RIV では最良だが RTLS では Mixed に劣る場合もある.
- **収束挙動** (Fig. 8): RTLS は RIV よりも収束が速い (mc_z の時間発展). RLS は初期収束は速いが最終精度が劣る.

### 検証結果に基づいた議論, 明らかになった課題はあるか？

(§V-B, V-C, §VI より) RTLS は RLS・RIV に対し, 負荷・軌道・加速度信号源の選択によらず一貫して最良の推定精度を示した. ただし以下の課題が残る:

- (§III) 関節角セットポイントに基づく条件数 κ_set と, 実験データに基づく条件数 κ_sens の間に大きな乖離がある (Table I: tr1 で κ_set = 7.72 vs κ_sens = 23.23). 軌道最適化はセットポイントベースで行われるが, 実際のセンサ信号では条件数が悪化する.
- (§V-C) 加速度信号源の最適な組合せは推定手法に依存する. RTLS では Mixed (JR3 線形加速度 + KF 角加速度) が最良だが, RLS/RIV では KF 信号のみが最良.
- 著者は推定時間 1.5 秒でのオンライン推定を実証したが, より長時間・より複雑な負荷での評価は行っていない. また, Approach 2 (オフセット直接推定) との組合せでの RTLS の性能評価は本論文では提示されていない.

---
## 自身の研究との関連

本論文は iparam_identification パッケージの TLS 実装の直接的な理論基盤. 現在の実装ではバッチ TLS (Golub-Van Loan SVD ベース) と再帰 TLS の両方が存在する. 2007 論文のリグレッサ構造・オフセット補償・差分法と, 本論文の TLS 定式化を組み合わせたものが現在のパイプライン. 実装上の重要な差異として, 現在のパイプラインは 4 手法 (OLS, TLS, OLS+bias, TLS+bias) を並列実行し, Approach 2 のバイアス項と TLS を組み合わせた Partial EIV (バイアス列を error-free とする TLS) を使用しているが, これは本論文では扱われていない拡張.

---
## 追加議論


---
## BibTex
<details>
<summary> Click to show/noshow the BibTex data </summary>

```bibtex
@inproceedings{kubus2008online,
  author    = {Kubus, Daniel and Kr{\"o}ger, Torsten and Wahl, Friedrich M.},
  title     = {On-Line Estimation of Inertial Parameters Using a Recursive Total Least-Squares Approach},
  booktitle = {Proc. of IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2008},
  pages     = {3845--3852},
  address   = {Nice, France},
  doi       = {10.1109/IROS.2008.4650772},
}
```
</details>
