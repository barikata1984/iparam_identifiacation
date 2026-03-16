# iparam_identification

UR5e の手首搭載 F/T センサを用いた、エンドエフェクタ負荷の慣性パラメータ（質量・重心・慣性テンソル）推定パッケージ。

## 概要

マニピュレータに取り付けた負荷を加振軌道上で動かし、F/T センサ値と運動学的加速度・角速度を融合して 10 個の慣性パラメータを推定する。

推定手法:
- **OLS** (Ordinary Least Squares) — 標準最小二乗法
- **Batch TLS** (Total Least Squares) — データ行列の誤差も考慮（Golub & Van Loan §6.3）
- **Recursive TLS** — オンライン推定（Kubus et al. 2008）

## パッケージ構成

```
iparam_identification/
├── scripts/                    # ROS ノード・CLI ツール
│   ├── batch_identifier.py         # バッチ同定 CLI（メインエントリポイント）
│   ├── tool0_kinematics_node.py    # Pinocchio ベース運動学パブリッシャ
│   ├── preview_excitation_trajectory.py  # 加振軌道 RViz プレビュー
│   ├── analyze_bias.py             # 同定結果の診断分析
│   ├── verify_regressor_matrix.py  # リグレッサ行列リアルタイム検証
│   └── ...
├── src/
│   ├── identifiers/
│   │   ├── tls.py                  # バッチ TLS ソルバ（3 スケーリングモード）
│   │   └── recursive_tls.py        # 再帰 TLS ソルバ（忘却係数付き）
│   └── utilities/
│       ├── dynamics_utils.py       # リグレッサ行列構築、bullet 演算
│       ├── tool0_kinematics.py     # Pinocchio ベース tool0 運動学
│       └── numerical_differentiator.py  # ローパスフィルタ付き数値微分
├── launch/
│   ├── batch_identifier.launch     # バッチ同定一式
│   ├── tool0_kinematics.launch     # 運動学ノード単体
│   └── preview_trajectory.launch   # 軌道プレビュー用 RViz
├── data/trajectories/              # 加振軌道 JSON
├── docs/                           # ドキュメント（ISSUES/TODO/LOGS + 参照文書）
├── literature/                     # 参考文献 PDF
└── results/                        # 同定結果出力（gitignore）
```

## 使い方

### バッチ同定

```bash
# 1. 運動学ノード + バッチ同定ノードを起動
roslaunch iparam_identification batch_identifier.launch

# 2. CLI の指示に従う:
#    Enter → F/T センサゼロ化 + 記録開始
#    (加振軌道を実行)
#    Enter → 記録停止
#    データ範囲を指定 → OLS/TLS 結果を比較 → 承認して publish
```

### 加振軌道プレビュー

```bash
# RViz で軌道を事前確認
roslaunch iparam_identification preview_trajectory.launch
rosrun iparam_identification preview_excitation_trajectory.py --speed 0.3
```

## 理論

### 推定方程式

Newton-Euler 方程式から導出されるリグレッサ形式:

```
[f; τ] = A(a, α, ω, g) · φ
```

- `A`: 6×10 リグレッサ行列（加速度・角速度から構築）
- `φ`: 10 次元パラメータベクトル `[m, mcx, mcy, mcz, Ixx, Iyy, Izz, Ixy, Iyz, Izx]`

### TLS のスケーリング行列

TLS は `min ||D[E|r]T||_F` を解く。重み行列 D, T の理論的意味:

- **T**: 各列（変数）のノイズレベルの逆数 — ノイズが小さい列をより信頼
- **D**: 各行（観測）の信頼度

現在の実装ではデータの標準偏差ベースのスケーリングを使用。ノイズベースのスケーリングへの改善を検討中（詳細: `docs/LOGS/log_tls_scaling.md`）。

## 参考文献

- Kubus, D., Kröger, T., & Wahl, F. M. (2008). On-line estimation of inertial parameters using a recursive total least-squares approach. IROS.
- Golub, G. H., & Van Loan, C. F. (2012). Matrix Computations, 4th ed., §6.3.
- Lynch, K. M., & Park, F. C. (2017). Modern Robotics, Chapter 8.
