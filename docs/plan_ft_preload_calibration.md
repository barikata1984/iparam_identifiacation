# ft_raw_wrench プリロードキャリブレーション計画

## Context

UR5e の `/wrench` トピック (`actual_TCP_force`) はブレーキリリース時に自動ゼロ化されるため、
慣性パラメータ同定に使う絶対的な F/T 値が得られない。
`ft_raw_wrench` (RTDE) はゼロ化されないが、構造的プリロード (~25000N) が含まれる。

**目的**: グリッパ等を外した無負荷状態で、6面体の各面方向にフランジを向けた 6 姿勢で
`ft_raw_wrench` を計測し、プリロードが姿勢に依存しない定数であることを実証する。
確認できれば、その平均値をプリロード定数として保存し、以降の同定で使用する。

## 実装物

### 1. 姿勢定義モジュール: `src/calibration/cube_poses.py`

6 姿勢の関節角度を定義する。ホーム姿勢（フランジ+Z = ベース-Z）は既知。
残り 5 姿勢は Pinocchio IK で算出するか、UR5e の構造を利用して解析的に求める。

**基準姿勢**: 励起軌道初期姿勢 `[90, -90, 90, -90, -90, 0]` deg
（`excitation_trajectory.json` の `trajectory[0].q`）
- 位置: [-0.13, 0.49, 0.49] m
- フランジ Z = [0, 0, -1] (ベース -Z)

**6 姿勢**:
| # | フランジ Z 方向 | 関節角度 |
|---|---|---|
| 1 | -Z (下向き) | **基準姿勢** [90, -90, 90, -90, -90, 0] deg |
| 2 | +Z (上向き) | 算出 |
| 3 | +X | 算出 |
| 4 | -X | 算出 |
| 5 | +Y | 算出 |
| 6 | -Y | 算出 |

姿勢算出方法:
- 基準姿勢の位置近傍で、フランジ Z 方向のみ変えた 5 姿勢を IK (Pinocchio) で求める
- IK 失敗時は wrist joints (J4, J5, J6) を調整して解析的に求める
- **全姿勢で安全範囲内 (関節リミット内、自己干渉なし) を確認**

### 2. RViz プレビュースクリプト: `scripts/preview_cube_poses.py`

既存の `preview_trajectory.launch` を活用。
`/preview/joint_states` に JointState メッセージを publish し、
6 姿勢を順番に表示する（各姿勢 3 秒間保持、Enter で次の姿勢に進む）。

- ROS ノードとして動作
- `cube_poses.py` から姿勢を読み込む
- 各姿勢でフランジ Z 方向のテキスト表示

**起動方法**:
```bash
roslaunch iparam_identification preview_trajectory.launch
# 別ターミナルで:
rosrun iparam_identification preview_cube_poses.py
```

### 3. 実機計測スクリプト: `scripts/calibrate_ft_preload.py`

ur_rtde ライブラリで直接ロボットを制御し、ft_raw_wrench を計測する。

**処理フロー**:
1. ur_rtde で接続 (RTDEControlInterface + RTDEReceiveInterface)
2. 各姿勢に moveJ で移動 (低速、5秒)
3. 2 秒静止待ち（振動収束）
4. 3 秒間 ft_raw_wrench をサンプリング (500Hz → ~1500 サンプル)
5. 各姿勢の平均・標準偏差を記録
6. 全 6 姿勢完了後に統計まとめを表示:
   - 姿勢間の平均値のばらつき (姿勢依存性の有無)
   - 各姿勢内のノイズ σ
   - 全姿勢の総合平均 = プリロード推定値
7. 結果を JSON に保存: `results/ft_preload_calibration_<timestamp>.json`

**安全対策**:
- 各姿勢移動前にユーザ確認 (Enter)
- moveJ は低速 (speed=0.5 rad/s, acceleration=0.3 rad/s²)
- グリッパ未装着であることをスクリプト開始時に警告表示

## ファイル構成

```
catkin_ws/src/iparam_identification/
├── src/calibration/
│   ├── __init__.py
│   └── cube_poses.py          # 6姿勢の関節角度定義
├── scripts/
│   ├── preview_cube_poses.py   # RViz プレビュー (ROS ノード)
│   └── calibrate_ft_preload.py # 実機計測 (ur_rtde 直接)
└── launch/
    └── preview_trajectory.launch  # 既存 (変更なし)
```

## 既存リソースの再利用

- `preview_trajectory.launch`: RViz + robot_state_publisher (変更不要)
- `Tool0KinematicsCalculator`: FK 検証に使用 (姿勢算出時)
- `ur_rtde` (pip installed): RTDEControlInterface.moveJ(), RTDEReceiveInterface.getFtRawWrench()
- URDF: `/root/osx-ur/underlay_ws/src/ur_python_utilities/ur_pykdl/urdf/ur5e.urdf`

## 検証手順

1. **姿勢算出の検証**: Pinocchio FK で各姿勢の tool0 回転行列を計算し、
   フランジ Z 列が期待方向 (±X, ±Y, ±Z) と一致することを確認
2. **RViz プレビュー**: `preview_cube_poses.py` で 6 姿勢を目視確認。
   自己干渉・不自然な姿勢がないか確認
3. **実機計測**: グリッパを外した状態で `calibrate_ft_preload.py` を実行。
   6 姿勢の ft_raw_wrench が一定であることを確認
