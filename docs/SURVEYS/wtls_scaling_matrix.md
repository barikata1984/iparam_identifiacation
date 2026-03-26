# Weighted TLS Scaling/Weighting Matrix Design

| Field | Value |
|-------|-------|
| Topic | Weighted TLS における重み行列 (D, T) の選択方法 |
| Date | 2026-03-25 |
| Papers | 26 (core focus) |
| Scope | TLS/EIV 回帰の重み行列設計、ML との接続、数値正規化 vs 統計的重み付け |

---

## Research Landscape Overview

Weighted Total Least Squares (WTLS) の重み行列選択に関する研究は、3つの独立した分野から
収束してきた。(1) 数値線型代数（Golub & Van Loan, Van Huffel & Vandewalle）による TLS の
計算理論、(2) 測地学（Schaffrin, Amiri-Simkooei, Fang）による実用的な WTLS アルゴリズム
開発、(3) 統計学（Fuller, Carroll）による Errors-in-Variables (EIV) モデルの推定理論である。

核心的な知見は単純である: **WTLS の重み行列は観測ノイズの共分散行列の逆行列とすべきであり、
この選択はガウスノイズ仮定下で最尤推定量 (ML) に一致する。** しかし、この処方箋の実用上の
困難は (a) ノイズ共分散が未知の場合の推定方法、(b) 共分散行列の構造制約（Kronecker 積）、
(c) 再帰的推定での online 更新、にある。

---

## Survey Findings

### Thesis

この分野の根本的な緊張は、**TLS のスケール非不変性**と**ノイズ共分散の未知性**の間にある。

Golub & Van Loan (1980) の基本 TLS は `min ||[E|r]||_F` を解くが、この Frobenius ノルムは
全要素を等分散と仮定する。変数間でスケールやノイズレベルが異なる場合（慣性パラメータ同定
では力 vs トルク vs 慣性モーメント）、この仮定は成り立たず、推定量は最適でなくなる。

重み行列 W = Σ^{-1}（ノイズ共分散の逆行列）を導入すれば ML 推定量が得られるが、これは
ノイズ共分散 Σ が既知であることを前提とする。Fuller (1987) が示すように、EIV モデルは
ノイズ分散比が未知のとき**同定不能**である。

### Foundation

調査した論文群に共通する技術的基盤は以下の3点である。

1. **SVD ベースの解法**: 基本 TLS は拡大行列 [A|b] の SVD から解析的に解ける
   (Golub & Van Loan 1980)。重み付き TLS では C = D[A|b]T の SVD を用いる。

2. **W = Σ^{-1} の処方箋**: WTLS で W をノイズ共分散の逆行列に設定すると、ガウスノイズ
   仮定下で ML 推定量に一致する。これは Kukush & Van Huffel (2004)、Markovsky & Van Huffel
   (2007)、Crassidis & Cheng (2019) で独立に確認されている。

3. **cofactor matrix による確率モデル記述**: 測地学では P = Q^{-1}（cofactor matrix の逆）
   を重み行列とする慣行が確立されており、WTLS にそのまま適用されている。

### Progress

| 時期 | 進展 | 克服された制約 |
|------|------|----------------|
| 1980 | 基本 TLS (Golub & Van Loan) | OLS の「A は正確」仮定を除去 |
| 1989 | GTLS (Van Huffel & Vandewalle) | 一部の列がエラーフリーの場合に対応 |
| 2004-06 | EW-TLS (Kukush; Markovsky et al.) | 要素ごとの異分散ノイズに対応、ML との等価性証明 |
| 2008 | WTLS (Schaffrin & Wieser) | 測地学向け解析的 WTLS（Kronecker 構造制約あり） |
| 2012 | SLS 再定式化 (Amiri-Simkooei & Jazaeri) | Kronecker 制約を除去、標準 LS 理論の全ツール適用可能に |
| 2013 | LS-VCE for EIV (Amiri-Simkooei) | ノイズ共分散が未知の場合のデータ駆動的推定 |
| 2014 | RGTLS + NCE (Rhode et al.) | 再帰的 TLS でのオンラインノイズ共分散推定 |
| 2017 | 特異共分散対応 (Amiri-Simkooei) | Q_A が特異（一部列がエラーフリー）の場合に対応 |
| 2019 | ML 解析 (Crassidis & Cheng) | 完全相関ノイズ下の ML 定式化 + Fisher 情報行列 |
| 2024 | 統一理論 (Hu, Fang & Zeng) | ランク不足 A + 特異 D を同時に許容 |
| 2024 | Enhanced RTLS (El-Sherbiny et al.) | 部分空間追跡 + ノイズ共分散適応の統合 |

### Gap

**未知ノイズ共分散下での WTLS 重み選択は未解決問題である。** Amiri-Simkooei (2013) の
LS-VCE は分散成分推定を可能にするが、推定可能な成分数は冗長度に制約される。
Rhode (2014) と El-Sherbiny (2024) は再帰的推定でのオンライン共分散推定を実現したが、
指数忘却を用いるため統計的整合性は犠牲になる。慣性パラメータ同定のように計測時間が
短い（1.5〜5秒）アプリケーションでは、共分散推定に十分なデータが得られない可能性がある。

**数値的正規化と統計的重み付けの混同が実務で広く存在する。** Golub & Van Loan の定式化で
D, T は自由パラメータであり、選択の指針を与えていない。Kubus (2008) は D, T を導入しながら
具体的な値を規定していない。その結果、列の標準偏差による正規化（数値的条件改善）が
ノイズベースの重み付け（統計的最適性）と混同されやすい。前者は SVD の数値安定性に寄与するが、
後者とは目的が異なる。

---

## Paper Catalogue

### Category A: TLS 基礎理論

基本 TLS から重み付き拡張に至る数学的基盤。

#### A1. Golub & Van Loan (1980)
- **Title**: An Analysis of the Total Least Squares Problem
- **Venue**: SIAM J. Numerical Analysis 17:883-893
- **DOI**: 10.1137/0717073
- **thesis**: A と b の双方に誤差を許容する SVD ベースの解法を提示。
- **core**: 拡大行列 [A|b] の SVD。最小特異値に対応する右特異ベクトルから解を構成。
- **diff**: OLS の「A は正確」仮定を除去。
- **limit**: 等分散・無相関ノイズを暗黙に仮定。重み行列やスケール不変性の議論なし。

#### A2. Van Huffel & Vandewalle (1991)
- **Title**: The Total Least Squares Problem: Computational Aspects and Analysis
- **Venue**: SIAM (Frontiers in Applied Mathematics, No. 9)
- **ISBN**: 0-89871-275-0
- **thesis**: TLS の計算・統計的側面を統一的に扱う初の単行本。
- **core**: SVD ベースアルゴリズム、truncated TLS、摂動理論。
- **diff**: 散在していた TLS の結果を統合。GTLS（一部列がエラーフリー）を導入。
- **limit**: 統計解析は i.i.d. ノイズ仮定に限定。重み付き拡張は未解決問題として提示。

#### A3. Van Huffel & Vandewalle (1989)
- **Title**: Analysis and Properties of the Generalized TLS Problem
- **Venue**: SIAM J. Matrix Analysis 10:294-315
- **DOI**: 10.1137/0610023
- **thesis**: 一部の列がエラーフリーの場合、基本 TLS は不必要に正確なデータを摂動する。
- **core**: A = [A1|A2] の分割 SVD。行列 C による列レベルの誤差構造エンコード。
- **diff**: 基本 TLS の全列等価仮定を除去。重み行列の前駆体。
- **limit**: 列レベルのスケーリングのみ、要素ごとの異分散には未対応。

### Category B: WTLS = ML 接続

重み行列の選択がノイズ共分散の逆行列であるべき根拠を与える論文群。

#### B1. Kukush & Van Huffel (2004)
- **Title**: Consistency of Elementwise-Weighted TLS Estimator
- **Venue**: Metrika
- **DOI**: 10.1007/S001840300272
- **Citations**: 72
- **thesis**: EW-TLS 推定量は異分散・行内相関ノイズ下で一致性を持つ。
- **core**: 行 i の重みは既知共分散 Σ_i の逆行列。コスト関数は行ごとの Mahalanobis 距離の和。
- **diff**: 従来の一致性結果は等分散・無相関に限定。異分散・相関ケースに拡張。
- **limit**: ノイズ共分散構造はスカラー倍を除いて既知でなければならない。

#### B2. Markovsky, Rastello, Premoli, Kukush & Van Huffel (2006)
- **Title**: The Element-Wise Weighted Total Least-Squares Problem
- **Venue**: Computational Statistics & Data Analysis 50(1):181-209
- **DOI**: 10.1016/j.csda.2004.07.014
- **Citations**: 105
- **thesis**: 異分散ノイズ下で基本 TLS は不一致。要素ごとの重み付けが一致性回復に必要十分。
- **core**: 重み w_{ij} = 1/σ_{ij}² を各要素に適用。交互変数法と Gauss-Newton 法。
- **diff**: GTLS は列レベル、EW-TLS は要素レベルの異分散に対応。
- **limit**: 局所最適解のみ。計算量は問題規模に依存。

#### B3. Markovsky & Van Huffel (2007)
- **Title**: Overview of Total Least-Squares Methods
- **Venue**: Signal Processing 87(10):2283-2302
- **DOI**: 10.1016/j.sigpro.2007.04.004
- **Citations**: ~700
- **thesis**: TLS は重み付き・構造化低ランク近似問題の特殊ケース。重み行列はノイズ共分散から導出すべき。
- **core**: 統一フレームワーク: `min ||vec(ΔD)||_W` subject to rank constraint。W=I が基本 TLS、W=Σ^{-1} が WTLS。
- **diff**: 統計（EIV）、数値線型代数（TLS）、信号処理（構造化近似）の視点を統合。
- **limit**: WTLS/STLS は閉形式解なし、局所最適化のみ。

#### B4. Crassidis & Cheng (2019)
- **Title**: Maximum Likelihood Analysis of the TLS Problem with Correlated Errors
- **Venue**: J. Guidance, Control, and Dynamics
- **DOI**: 10.2514/1.G003815
- **Citations**: 14
- **thesis**: TLS の正しい統計的枠組みは ML であり、完全相関ノイズ下でも最適推定と誤差共分散を提供する。
- **core**: ML コスト関数: J = vec(Δ)^T R^{-1} vec(Δ)。R は vec([A,B]) の完全共分散行列。Fisher 情報行列による Cramer-Rao 下界。
- **diff**: 2014年の Crassidis 論文は無相関ノイズのみ。完全相関ケースに拡張。
- **limit**: ノイズ共分散 R は既知（またはよく推定されている）必要あり。

#### B5. White, Tan & Hammond (2006)
- **Title**: Analysis of the ML, TLS and PCA Approaches for FRF Estimation
- **Venue**: J. Sound and Vibration
- **DOI**: 10.1016/J.JSV.2005.04.029
- **Citations**: 38
- **thesis**: FRF 推定において PCA/TLS/ML は同一問題の3つの視点。ML のみが不等ノイズレベルを自然に扱える。
- **core**: 重み = 出力ノイズ分散 / 入力ノイズ分散の比。比が1のとき基本 TLS に退化。
- **diff**: 基本 TLS/PCA は等ノイズを暗黙に仮定。ML 導出による一般化 TLS がこれを修正。
- **limit**: limit not available

### Category C: 測地学 WTLS アルゴリズム

cofactor matrix Q に基づく実用的な WTLS の定式化と解法。

#### C1. Schaffrin & Wieser (2008)
- **Title**: On Weighted TLS Adjustment for Linear Regression
- **Venue**: J. Geodesy 82:415-421
- **DOI**: 10.1007/s00190-007-0190-9
- **Citations**: 268
- **thesis**: 異分散・相関観測に対する初の解析的 WTLS 解法。
- **core**: Q_A = Q_0 ⊗ Q_x（Kronecker 積構造）の制約下で閉形式反復解。P_y = Q_y^{-1}。
- **diff**: 基本 TLS の等分散仮定を除去。
- **limit**: Kronecker 積構造に限定。一般的な Q_A には対応不可。

#### C2. Amiri-Simkooei & Jazaeri (2012)
- **Title**: WTLS Formulated by Standard Least Squares Theory
- **Venue**: J. Geodetic Science 2(2):113-124
- **DOI**: 10.2478/v10156-011-0036-5
- **thesis**: WTLS を標準 LS の拡大パラメトリックモデルとして再定式化。LS の全ツールが直接利用可能。
- **core**: EIV モデルを拡大未知数（パラメータ + A の修正量）のパラメトリックモデルに変換。P = Q^{-1}（一般的 Q、Kronecker 制約なし）。
- **diff**: Schaffrin & Wieser (2008) の Kronecker 制約を除去。
- **limit**: limit not available

#### C3. Fang (2013)
- **Title**: WTLS: Necessary and Sufficient Conditions
- **Venue**: J. Geodesy 87:733-749
- **DOI**: 10.1007/s00190-013-0643-2
- **thesis**: WTLS 最適性の必要十分条件を Hessian 解析により導出。
- **core**: 3つの解法の比較（反復正規方程式、Gauss-Helmert、数値解析）。一般的分散行列、固定/確率パラメータの統一扱い。
- **diff**: Schaffrin & Wieser (2008)、Mahboub (2012) は必要条件のみ。十分条件を追加。
- **limit**: Hessian 計算の追加コスト。

#### C4. Amiri-Simkooei (2013)
- **Title**: Application of LS-VCE to Errors-in-Variables Models
- **Venue**: J. Geodesy 87:935-944
- **DOI**: 10.1007/s00190-013-0658-8
- **thesis**: ノイズ共分散が未知の場合、LS-VCE で分散成分をデータ駆動的に推定可能。
- **core**: Q = Σ_k (σ_k Q_k) の分解。LS-VCE 公式 σ̂ = N^{-1} l の反復適用（2-4回）。
- **diff**: 標準 WTLS は Q が既知を前提。未知の場合のデータ駆動推定を提供。
- **limit**: 推定可能な成分数は冗長度に制約。Kronecker 仮定は全観測型に同一パターンを要求。

#### C5. Mahboub (2012)
- **Title**: On Weighted TLS for Geodetic Transformations
- **Venue**: J. Geodesy 86:359-367
- **DOI**: 10.1007/s00190-011-0524-5
- **Citations**: 113
- **thesis**: Kronecker 制約なしの WTLS アルゴリズム。
- **core**: 完全逆分散行列 D^{-1} を直接使用。P = D^{-1}。
- **diff**: Schaffrin & Wieser (2008) の Kronecker 制約を除去。
- **limit**: limit not available

#### C6. Amiri-Simkooei, Zangeneh-Nejad & Asgari (2016)
- **Title**: On the Covariance Matrix of WTLS Estimates
- **Venue**: J. Surveying Engineering 142(3)
- **DOI**: 10.1061/(ASCE)SU.1943-5428.0000153
- **Citations**: 33
- **thesis**: WTLS 推定値の共分散は、SLS 正規行列の逆行列で十分近似可能。
- **core**: 3戦略の比較: 正規行列逆、非線形誤差伝播、Monte Carlo。
- **diff**: WTLS の共分散推定戦略の初の体系的比較。
- **limit**: 高ノイズ環境では正規行列近似が劣化。

#### C7. Amiri-Simkooei (2017)
- **Title**: WTLS with Singular Covariance Matrices
- **Venue**: J. Surveying Engineering 143(4)
- **DOI**: 10.1061/(ASCE)SU.1943-5428.0000239
- **Citations**: 17
- **thesis**: SLS ベース WTLS を特異（ランク不足）Q_A と制約付き問題に拡張。
- **core**: 特異 Q_A（一部列がエラーフリー）を許容。制約 Cx=d を SLS 系に組込み。
- **diff**: 2012年版は Q_A 非特異を仮定。特異ケースに対応。
- **limit**: limit not available

#### C8. Hu, Fang & Zeng (2024)
- **Title**: Toward a Unified Approach to the TLS Adjustment
- **Venue**: J. Geodesy 98, art. 75
- **DOI**: 10.1007/s00190-024-01882-x
- **thesis**: ランク不足 A と特異 D を同時に許容する統一 WTLS 理論。
- **core**: 一般化逆行列による拡張。P = D^{-}（g-inverse）。
- **diff**: 従来の全 WTLS はフルランク A または非特異 D を前提。両方の制約を除去。
- **limit**: limit not available

#### C9. Wurm (2021)
- **Title**: A Universal and Fast Method for WTLS with Correlated Coefficients
- **Venue**: Measurement Science and Technology 32:125011
- **DOI**: 10.1088/1361-6501/ac32ec
- **Citations**: 8
- **thesis**: A-b 間の交差相関を含む完全一般の重み行列 W を扱う実用的 WTLS。
- **core**: W = [W_A, W_Ab; W_Ab^T, W_b] の分割構造。Cholesky 分解 + 解析的初期値。
- **diff**: 測地学 WTLS は A-b 交差相関を通常無視。完全一般ケースに対応。
- **limit**: 局所最適化のみ。Hessian ベース共分散は追加コスト。

### Category D: 再帰的 TLS とノイズ共分散推定

オンライン推定における重み行列の動的決定。

#### D1. Kubus, Kröger & Wahl (2008)
- **Title**: On-Line Estimation of Inertial Parameters Using RTLS
- **Venue**: IROS 2008
- **DOI**: 10.1109/IROS.2008.4650672
- **Citations**: 67
- **thesis**: Brand の incremental SVD を用いた再帰的 TLS で慣性パラメータを ~1.5秒で推定。
- **core**: N_k = D[A_k, (f_k; τ_k)]T。左特異ベクトルのみ更新。φ̂ = -t_{ii} w_{i,n+1} / (t_{n+1,n+1} w_{n+1,n+1})。
- **diff**: バッチ OLS はデータ行列のノイズを無視。RTLS は RLS, RIV を精度で上回る。
- **limit**: **D, T の具体的な値を規定していない。** 実験では事実上 D=I, T=I（等分散仮定）。

#### D2. Rhode, Bleimund & Gauterin (2014)
- **Title**: Recursive Generalized TLS with Noise Covariance Estimation
- **Venue**: IFAC Proceedings 47:4637-4643
- **DOI**: 10.3182/20140824-6-ZA-1003.01773
- **thesis**: RGTLS にノイズ共分散推定器 (NCE) を並列実行させることで、共分散未知でも WTLS が可能。
- **core**: 2モジュール構成: RGTLS + NCE。NCE は指数忘却で共分散を再帰更新し、RGTLS の重みを動的に設定。
- **diff**: Kubus (2008) を含む従来 RTLS は共分散既知（または単位行列）を仮定。
- **limit**: 指数忘却により統計的一致性を犠牲。忘却係数のチューニングに依存。

#### D3. El-Sherbiny, Mercère, Arvis & Biesse (2024)
- **Title**: Enhanced Recursive TLS with Subspace Tracking and Noise Covariance Estimation
- **Venue**: IEEE Control Systems Letters
- **DOI**: 10.1109/LCSYS.2024.3408003
- **Citations**: 1
- **thesis**: 部分空間追跡とノイズ共分散適応を WTLS に統合し、時変システムの追跡性能を向上。
- **core**: 3コンポーネント: WTLS + 部分空間追跡（SVD 更新の代替）+ ノイズ共分散適応。
- **diff**: Rhode (2014) の incremental SVD を計算量の少ない部分空間追跡に置換。共分散適応をより密に統合。
- **limit**: limit not available

### Category E: EIV 統計理論（教科書）

#### E1. Fuller (1987)
- **Title**: Measurement Error Models
- **Venue**: Wiley
- **ISBN**: 0-471-86187-1
- **thesis**: EIV モデルの一致推定にはノイズ分散比の知識が必須。
- **core**: 信頼度比 λ = σ²_star / (σ²_η + σ²_star)。既知ノイズ分散の場合、観測共分散からノイズ共分散を差し引いて一致推定。
- **diff**: EIV 推定の統一的・厳密な統計的枠組みを提供。
- **limit**: ノイズ分散比が未知のとき、モデルは同定不能。補助情報が必要。

#### E2. Carroll, Ruppert, Stefanski & Crainiceanu (2006)
- **Title**: Measurement Error in Nonlinear Models: A Modern Perspective (2nd ed.)
- **Venue**: CRC Press
- **ISBN**: 978-1-58488-633-4
- **thesis**: 非線形モデルの測定誤差には線形 EIV 以上の専用補正法が必要。
- **core**: Regression Calibration, SIMEX, 道具変数、Score 関数法の4手法。
- **diff**: Fuller (1987) の線形 EIV 理論を非線形モデルに拡張。
- **limit**: SIMEX はノイズ分散の知識が必要。高度に非線形なモデルでは近似が破綻。

### Category F: TLS 手法比較

#### F1. Markovsky (2010)
- **Title**: Total Least Squares Methods
- **Venue**: WIREs Computational Statistics 2(2):212-220
- **DOI**: 10.1002/wics.65
- **thesis**: TLS の分類法: 基本 (W=I)、重み付き (W=Σ^{-1})、構造化（Toeplitz 等の制約）。
- **core**: 統一的レビュー。WTLS/STLS は閉形式解なし、局所最適化が必要。
- **diff**: 統計・数値線型代数・信号処理の視点を統合するレビュー。
- **limit**: WTLS/STLS は局所最適解のみ。

#### F2. Zhou, Kou, Li & Fang (2017)
- **Title**: Comparison of Structured and Weighted TLS Adjustment Methods
- **Venue**: J. Surveying Engineering
- **DOI**: 10.1061/(ASCE)SU.1943-5428.0000190
- **Citations**: 16
- **thesis**: 線形構造 EIV で STLS と WTLS を直接比較。
- **core**: CTLS, STLN, WTLS の3手法を測地学問題で比較。
- **diff**: STLS と WTLS を別個の研究系統から統合比較。
- **limit**: limit not available

#### F3. Liu, Li, Hendeby & Gustafsson (2023)
- **Title**: WTLS for Quadratic Errors-in-Variables Regression
- **Venue**: EUSIPCO 2023
- **DOI**: 10.23919/EUSIPCO58844.2023.10289806
- **Citations**: 1
- **thesis**: 二次リグレッサの EIV では線形誤差伝播ベースの重みが不正確。解析的モーメントから正しい重みを導出。
- **core**: ガウス確率変数の解析的モーメントから二次データ行列の誤差統計量を導出し、適切な重み行列を構成。
- **diff**: 標準 WTLS は線形関係を仮定。二次以上の多項式リグレッサに拡張。
- **limit**: シミュレーションのみ。ガウスノイズ仮定。

---

## Survey Methodology

### Search Log

| # | Source | Query | Results | Notes |
|---|--------|-------|---------|-------|
| 1 | WebSearch | "weighted total least squares" "maximum likelihood" column weighting | 10 | WTLS=ML 接続確認。Markovsky & Van Huffel 2007 発見 |
| 2 | WebSearch | "weighted total least squares" noise covariance scaling matrix | 10 | W = σ^{-2} Σ^{-1} 確認 |
| 3 | WebSearch | Van Huffel "total least squares" weighting matrix choice | 10 | SIAM book, Liu 2017 発見 |
| 4 | WebSearch | Markovsky "structured total least" weighting | 10 | EW-TLS, STLS 論文群 |
| 5 | WebSearch | WTLS geodetic surveying weighting matrix | 10 | Mahboub 2012, Malissiovas 2020, Schaffrin & Wieser 発見 |
| 6 | WebSearch | Schaffrin Wieser "weighted total least squares" | 10 | 基盤論文確認 |
| 7 | WebSearch | Amiri-Simkooei "weighted total least squares" covariance | 10 | VCE, 特異共分散論文群 |
| 8 | WebSearch | Fang "weighted total least squares" algorithm | 10 | 必要十分条件、制約付き WTLS |
| 9 | WebSearch | Crassidis "Maximum Likelihood Analysis" TLS 2019 | 10 | JGCD 論文確認 |
| 10 | WebSearch | Rhode Bleimund "recursive generalized total least squares" | 10 | IFAC 2014 確認 |
| 11 | WebSearch | Fuller "measurement error models" textbook | 10 | 教科書確認 |
| 12 | WebSearch | Golub Van Loan "total least squares" 1980 | 10 | 基盤論文確認 |
| 13 | Semantic Scholar | "weighted total least squares scaling matrix" | 20 | Amiri-Simkooei, Wang, Fang 論文群。citation counts 取得 |
| 14 | Semantic Scholar | "total least squares maximum likelihood noise covariance" | 20 | Crassidis & Cheng 2019 (14 citations) 確認 |
| 15 | Semantic Scholar | "errors in variables model weighting noise covariance" | 20 | El-Sherbiny 2024 発見 |
| 16 | Semantic Scholar | "Schaffrin weighted total least squares cofactor" | 15 | Schaffrin 論文群 citation counts |
| 17 | Semantic Scholar | "generalized total least squares column scaling" | 15 | Van Huffel 1989 (210 citations) 発見 |

Duplicates removed: ~15

### DOI Resolution Log

| Paper | Original ID | Resolved DOI | Method |
|-------|------------|--------------|--------|
| Kubus 2008 | IROS conference | 10.1109/IROS.2008.4650672 | DBLP + S2 |
| Kukush 2004 | Metrika journal | 10.1007/S001840300272 | S2 |
| Markovsky 2006 | CSDA journal | 10.1016/j.csda.2004.07.014 | DBLP + S2 |

---

## Practical Implications for Inertial Parameter Identification

### 現在の実装への適用

本調査から、慣性パラメータ同定の TLS スケーリングに対して以下が導かれる:

1. **理論的に正しい T の選択**: `t_i = 1/σ_noise_i`（列 i のノイズ標準偏差の逆数）。
   これにより WTLS = ML（ガウスノイズ下）となる (B1-B4)。

2. **現在の `COLUMN_ONLY` モード**: `t_i = 1/std_data_i` は列スケール正規化であり、
   ノイズベースの重み付けではない。数値的条件改善には寄与するが、統計的最適性は持たない。

3. **ノイズ推定法**: 提案済みの diff ベース推定 `σ_noise = std(diff(col)) / √2` は、
   測地学の LS-VCE (C4) の簡易版として正当化可能。500Hz サンプリングでは信号成分が
   diff で消えるため、ノイズの分離が可能。

4. **D（行重み）の扱い**: 測地学文献では P_y = Q_y^{-1} として観測の信頼度を反映するが、
   慣性パラメータ同定では各時刻の F/T 測定の信頼度が一様と仮定できるなら D=I で妥当。

5. **再帰的 TLS への拡張**: Rhode (2014) と El-Sherbiny (2024) の NCE アプローチは、
   再帰的 TLS にノイズ共分散推定を統合する具体的な方法を提供する。

### 命名の改善提案

| 現行 | 実態 | 提案 |
|------|------|------|
| `NONE` | 無重み | `IDENTITY`（明示的に W=I） |
| `COLUMN_ONLY` | 列スケール正規化 | `DATA_VARIANCE`（data std による正規化） |
| `FULL` | 列+行スケール正規化 | 削除候補（行の std に理論的根拠なし） |
| (未実装) | ノイズベース ML 重み | `NOISE_VARIANCE`（diff ベースノイズ推定） |
