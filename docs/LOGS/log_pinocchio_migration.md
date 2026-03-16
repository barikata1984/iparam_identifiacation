# Log: Pinocchio ベース運動学への移行

## 2026-03 (完了): ur_pykdl → Pinocchio 移行

既存の `wrist_end_kinematics_node.py` (ur_pykdl ベース) から pinocchio ベースの `tool0_kinematics.py` へ移行した。

### 背景

ur_pykdl ベースの問題点:
- 内部実装の詳細が不明確
- base frame → tool0 frame への座標変換が手動実装
- Coriolis 項の計算が正しいか未検証

Pinocchio ベースの利点:
- 広く使われテストされたライブラリ
- `getFrameClassicalAcceleration(LOCAL)` で直接 tool0 の classical acceleration を取得
- LOCAL/WORLD/LOCAL_WORLD_ALIGNED のフレーム指定が明確

### 実装フェーズ

| Phase | 内容 | 状態 |
|-------|------|------|
| Phase 1 | `tool0_kinematics.py` の `compute_with_gravity()` 追加 | 完了 |
| Phase 2 | `tool0_kinematics_node.py` の作成 | 完了 |
| Phase 3 | Launch ファイルの更新 | 完了 |
| Phase 4 | 検証 | 基本検証完了 |

### 重要な発見

#### Wrench の符号

当初 `batch_identifier.py` で wrench を反転していた（反力→作用力の変換のため）が、UR の内蔵 F/T センサは payload がセンサーに加える力を直接報告するため反転は不要。

#### Proper Acceleration

慣性パラメータ同定では F/T センサが測定する力は「適正加速度」に対応:
```
a_proper = a_kinematic + g_local
```
pinocchio で kinematic acceleration を取得後、重力を tool0 frame に変換して加算。

#### OLS vs TLS

TLS は異常値を出すことがあった（370g 物体で m≈0.88kg）。OLS は m≈0.31kg（17% 誤差）。
→ 現状では OLS を使用。

### 検証結果（370g 物体、静止状態）

| 項目 | 期待値 | 実測値 (OLS) | 誤差 |
|------|--------|------------|------|
| 質量 m | 0.370 kg | 0.307 kg | -17% |
| Matrix rank | 10 | 10 | - |

### 変更ファイル一覧

| ファイル | 変更内容 |
|---------|---------|
| `src/utilities/tool0_kinematics.py` | `compute_with_gravity()` メソッド追加 |
| `scripts/tool0_kinematics_node.py` | 新規作成 |
| `scripts/batch_identifier.py` | kinematics_ns パラメータ追加、wrench 反転削除、OLS 使用 |
| `launch/tool0_kinematics.launch` | 新規作成 |
| `launch/batch_identifier.launch` | `use_pinocchio` フラグ追加 |
