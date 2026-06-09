# FT 300-S 統合ログ

> 開始: 2026-06-05

## 目的

外付け Robotiq FT 300-S を慣性パラメータ同定パイプラインで使用可能にする.
内蔵 FT (I-11) の分解能制約を外部センサで回避する.

## 2026-06-05: 初期統合と診断

### 実施した変更

1. **`replay_excitation_trajectory.py`**: `speed` パラメータ追加(軌道再生速度スケーリング)
2. **`replay_excitation_trajectory.launch`**: `speed` 引数追加, `n_laps` デフォルトを 1 に変更
3. **URDF 修正** (`robotiq_ft300.urdf.xacro`):
   - `ft300_mounting_plate -> ft300_sensor` joint: `rpy=(0, pi, 0)` → `rpy=(0, 0, 0)`(フレーム方向を tool0 と揃え)
   - `ft300_sensor -> robotiq_ft_frame_id` joint: `rpy=(0, pi, -pi/2)` → `rpy=(0, 0, 0)`(同上)
   - mesh の回転を visual/collision `<origin>` に移動
   - CoM を新フレーム基準に再計算: `(0, -6, -13.5)mm` → `(0, 6, 13.5)mm`
4. **regressor フレーム切替**: `ft_sensor:=ft300s` のとき `robotiq_ft_frame_id` で運動学を計算
   - `Tool0KinematicsCalculator`: `frame_name` パラメータ追加
   - `IdentificationPipeline`: `frame_name` パラメータ追加
   - pinocchio 用 URDF 生成: `data/urdf/ur5e_ft300s_robotiq85.urdf`
   - グリッパ関節のゼロパディング(nq=12 対応)
5. **FT 300-S 用 URDF/フレーム設定**: `_FT_SENSOR_CONFIG` dict で `ft_sensor_kind` に応じて自動切替

### 同定結果と診断

340g 物体を把持して `traj.json`(5 秒, 501 ステップ)で同定.

| 手法 | m_total [kg] | Fz bias [N] | 備考 |
|------|-------------|-------------|------|
| OLS | -0.154 | — | |
| OLS+bias | -0.359 | +7.16 | 質量が負 |

### FTA による調査

1. **符号規約**: FT 300-S と UR `wrench_raw` の相関が負 → 符号反転を導入 → m=+0.341 に改善. ただし後に `wrench_raw` が未校正カウント値(N ではない)と判明, start pose での静的比較で内蔵 FT と FT 300-S は同極性(Fz 正)を確認 → **符号反転を revert**
2. **条件数**: S_aug の cond=17.1(内蔵 FT: 28.7). 問題なし
3. **時間同期**: wrench を -20~+20 フレームシフトしても mass 変化なし. 問題なし
4. **合成データ検証**: 既知 φ(m=1.265) + 既知 bias で wrench を合成 → パイプライン投入 → OLS+bias が **完璧に復元**(誤差 0). **パイプラインにバグなし**

### 未解決: OLS+bias total mass が ~0.34 kg(期待 ~0.91 kg)

- 合成データでは正しく動作するため, パイプラインの実装は正しい
- 内蔵 FT でも正常(skip_teleop: m = 1.41, 力 R² > 0.9)
- FT 300-S 実データでのみ total mass が物体質量相当(~0.34)にしかならない

## 2026-06-05: I-12 深層診断

### 除外した仮説

1. **符号規約不一致**: レンチ符号反転では R² は改善しない (単に推定パラメータの符号が反転するだけ)
2. **フレーム回転**: Rz(±90°), Rz(180°), Ry(180°) を含む 10 通りの wrench 変換を網羅的に試行. raw が最良で回転は全て悪化. センサ軸マーキングを tool0 に揃えて取り付けたため回転不要
3. **Rz(+90°) (Robotiq マニュアルのデフォルト)**: デフォルト取り付けでは sensor_x = tool0_y だが, ユーザーが 90° 回転して取り付け済み

### チャネル別 R² 分析 (08-40 データ, OLS+bias)

| チャネル | R² | 評価 |
|---------|-----|------|
| Fx | 0.41 | 不良 |
| Fy | 0.18 | 非常に不良 |
| Fz | 0.61 | 中程度 |
| Tx | 0.72 | 良 |
| Ty | 0.75 | 良 |
| Tz | -1.37 | モデルが平均以下 |

力チャネルの R² が低く, トルクチャネルは比較的良い — 力に系統的な欠損あり.

### 有力仮説: FT 300-S ファームウェアの重力補償

FT 300-S は内蔵加速度計による重力補償機能を持つ (Robotiq マニュアル):
- 3 姿勢キャリブレーションでツール質量と CoG を推定
- 結果をセンサ内に永続保存
- 以降の出力からツール重量の影響を自動除去

**検証**: 補償質量 m_comp を力チャネルに加算するスイープ:

| m_comp [kg] | m_est [kg] | mean R² | 評価 |
|-------------|-----------|---------|------|
| 0.00 | +0.34 | 0.218 | 現状 |
| 0.40 | +0.74 | 0.668 | |
| 0.60 | +0.94 | 0.774 | |
| **0.85** | **+1.19** | **0.816** | **最良** |
| 1.05 | +1.40 | 0.785 | |

m_comp ≈ 0.85 kg で R² が 0.22 → 0.82 に劇的改善, m_est ≈ 1.19 に回復.

力チャネルからグリッパ質量分の重力信号が欠損していることを意味する. トルクチャネルは補償の影響が異なる (CoG 依存) ため, 力とトルクの間で不整合が生じ, 全体の推定を劣化させる.

### 次ステップ

- [ ] `ft_sensor:=ft300s skip_teleop:=true` でグリッパのみの FT 300-S データを取り,
  内蔵 FT の同一軌道データ (06-34: m = 1.41, Fz range = 15.3 N) と力の変動幅を比較
  → ファームウェア重力補償の有無を実データで確定
- [ ] 補償確定の場合: (a) 無負荷でセンサ再キャリブレーション (重力補償を事実上無効化),
  または (b) m_comp を補正パラメータとしてパイプラインに組み込む
- [ ] `n_laps:=3` で再試行 (データ量を増やす)

## 2026-06-08: 文献ノート整理

Kubus et al. 2007/2008 の論文サマリノートを作成し、`literature/papers/` に配置.
元 PDF も `papers/{citekey}/main.pdf` に移動し、`literature/` 直下の重複を削除.

- `literature/papers/Kubus-IROS2007-Rigid_Object_Recognition/`
- `literature/papers/Kubus-IROS2008-Recursive_Total_Least-Squares/`

### 実装と論文の対応整理

4 手法 (OLS, TLS, OLS+bias, TLS+bias) と論文の構成要素のマトリクスを作成:

- 共通基盤: リグレッサ V (2007 eq. 1–5), 差分法 (2007 eq. 27–28)
- 誤差モデル: OLS (2008 eq. 16) vs TLS (2008 eq. 23)
- オフセット補償: zero_ftsensor (Approach 1 部分採用, V_ginit 未実装) + Approach 2 (bias 列拡張)
- 実装独自拡張: TLS+bias の Partial EIV (バイアス列を error-free 扱い)
