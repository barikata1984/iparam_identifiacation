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

### 未解決: OLS+bias total mass が ~0.34 kg(期待 ~1.27 kg)

- 合成データでは正しく動作するため, パイプラインの実装は正しい
- 実データでのみ total mass が物体質量相当(~0.34)にしかならない
- FT 300-S はグリッパ装着状態でゼロ点取得 → 測定値にグリッパ重力のゼロ姿勢分がオフセットとして乗る
- Kubus Approach 2(bias 同時推定)はこのオフセットを吸収し, φ に全質量を復元するはず
- **原因未特定** — 次セッションで実データのデバッグを継続

### 次ステップ

- [ ] 合成データにノイズ/ゼロ点オフセットを加えて, どの条件で推定が壊れるか調査
- [ ] FT 300-S の生データ(ゼロ前)を記録して, ゼロ後の値と比較
- [ ] 内蔵 FT の `actual_TCP_force` が payload 補償済みである影響を定量評価
- [ ] `n_laps:=3` で再試行(データ量を増やす)
