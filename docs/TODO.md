# TODO — マイクロタスク

> **最終更新**: 2026-03-15

---

## 推定精度修正（I-1, I-2）

- [x] Step 1: F/T 再ゼロ化を全箇所で除去 (I-1)
  - Phase 1: `controller.deactivate()` → teleop pub + `activate_joint_trajectory_controller()` に置換
  - Phase 3: `deactivate_compliance()` → `activate_joint_trajectory_controller()` に置換
  - Phase 3: 明示的 `zero_ft_sensor()` を削除
  - ドライバ起動時（物体なし）のゼロ点を維持する設計に変更
- [ ] Step 2: リグレッサ行列 Row 5, 6 を修正 (I-2)
  - `wrist_end_kinematics_utils.py` の Row 5 (Ny) と Row 6 (Nz) の交差慣性項を修正
  - `dynamics_utils.py` (Lynch & Park bullet 演算子) の正しい実装を参照
  - 検証テスト追加（omega=[1,0,0] 等で dynamics_utils と一致確認）
- [ ] Step 3: メッセージ同期の排除 (I-4)
  - recording callback 内で `Tool0KinematicsCalculator` を直接呼び出す
  - `/joint_states` + `/wrench` の 2 トピック同期のみに簡素化
  - kinematics node の 5 トピック subscribe を廃止
- [ ] Step 4: 実機で再推定し、120g 物体に対して正常な推定値を確認

---

## TLS スケーリング改善（I-3）

- [ ] `ScalingMode.NOISE_BASED` を `tls.py` に実装（diff ベースの T + D=I）
- [ ] 既存の 3 モードと並列比較実験（OLS との乖離が改善されるか検証）
- [ ] 効果確認後、再帰的 TLS にもスケーリングを組み込む
