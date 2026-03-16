# Log: 推定精度調査

## 2026-03-15: 推定値異常の原因調査

120g 物体に対して 180g / 10g / 負の質量を推定する問題を調査した。

### 発見1: F/T センサの再ゼロ化（主因）

`replay_excitation_trajectory.py` Phase 3 で物体把持後に F/T センサが再ゼロ化される。

- `deactivate_compliance()` (line 232) が内部で `zero_ft_sensor()` を呼び出す
- 明示的に `zero_ft_sensor()` (line 248) でも再ゼロ化
- UR の `zero_ftsensor` は呼出時の読み値を定数オフセットとして全読み値から差し引く
- 物体把持状態でゼロ化すると `F_measured(q) = F_true(q) - F_true(q0)` となり、リグレッサが仮定する `F = m*a_proper` との間に定数バイアス `-m*R(q0)^T*g` が生じる
- 120g 物体でバイアス ~1.18 N、グリッパ重量を含めると ~10 N — ペイロード信号と同等以上
- 手順書 `docs/excitation_replay_procedure.md` の「既知の問題」で既に指摘されていたが未修正

### 発見2: リグレッサ行列 Row 5, 6 の交差慣性項バグ（副因）

`wrist_end_kinematics_utils.py` の `get_regressor_matrix()` で Row 4 (Nx) は正しいが Row 5 (Ny) と Row 6 (Nz) に係数入れ替えバグがある。

**Row 5 (Ny)**: Iyy <-> Ixy スワップ、Iyz <-> Izx スワップ（2 組）
**Row 6 (Nz)**: Ixy/Iyz/Izx の循環置換 + Ixy 係数の変数誤り (wx^2 -> wz^2)

検証方法:
- Newton-Euler 手導出との比較
- `dynamics_utils.py` (Lynch & Park "bullet" 演算子ベース) の正しいリグレッサとの比較
- テストケース omega=[1,0,0], alpha=[0,0,0] で -1 が Izx (col 9) にあるべきところ Iyz (col 8) に出現

`dynamics_utils.py` に正しい実装が存在するが、`tool0_kinematics_node.py` は `wrist_end_kinematics_utils.py` (バグ版) を使用している。

### 発見3: TLS スケーリング（既知, 軽微）

`docs/LOGS/log_tls_scaling.md` で文書化済み。データ分散ではなくノイズ分散を使うべき。

## 2026-03-15: 55g 推定の原因調査（I-1, I-2 修正後）

I-1, I-2 修正後も 120g 物体に対して 55g と推定。追加調査を実施。

### 棄却: ツール重量姿勢バイアス仮説

- ホーム姿勢と励起開始姿勢は wrist_3 のみ 90° 差（z 軸回転）
- Pinocchio で計算: 両姿勢の g_local は完全一致 [0, 0, 9.81] m/s²
- ツール重量バイアス = 0 N → **この仮説は棄却**

### 発見: regressor と la の不整合（真の原因）

記録データの regressor mass 列と la (proper acceleration) が完全不一致:
```
Frame 0: la = [0.148, -9.390,  0.032]  (gravity in -y)
         reg col0 = [0.262, -0.482, 9.878]  (gravity in +z)
```

同じコールバック内で `la` から `regressor` を構築しているため、本来一致するはず。
不一致の原因: `ApproximateTimeSynchronizer` が `allow_headerless=True` で
headerless メッセージ (`Vector3`, `Float64MultiArray`) を受信時刻ベースで同期しているため、
500Hz のコールバック間でメッセージが混在している。

### 影響

- 各フレームの regressor / wrench / la が異なる時刻のデータの混合物
- `S * φ = W` の関係が成立せず、同定は原理的に不可能
- フレームレートも 49 Hz（500Hz の 1/10）で大量のフレーム欠損

### 対処方針

kinematics node 内で全データを単一メッセージとして publish するか、
recording node 内で直接 kinematics 計算を実行する設計に変更する必要がある。

## 2026-03-15: I-1 修正

`replay_excitation_trajectory.py` の全 F/T 再ゼロ化箇所を除去。

- Phase 1: `controller.deactivate()` → teleop pub 停止 + `arm.activate_joint_trajectory_controller()` に置換
- Phase 3: `robot.deactivate_compliance()` → `arm.activate_joint_trajectory_controller()` に置換
- Phase 3: 明示的 `robot.zero_ft_sensor()` + sleep を削除
- osx_bilateral 側のコードは変更なし（`_arm` / `_teleop_active_pub` への直接アクセスで回避）
