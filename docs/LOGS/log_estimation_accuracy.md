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

### 棄却した仮説

1. **ツール重量姿勢バイアス**: ホームと励起開始は g_local 完全一致 [0,0,9.81]。
   軌道中の変動 (平均 10°, cos=0.97) では理論上 m_ols ≥ 120g で方向が逆。棄却。
2. **la/regressor の不整合 (q=0 汚染)**: 19/69 フレームで kinematics node が非 UR の
   /joint_states を処理し q=0 で計算。不整合は実在するが、汚染フレーム除去後も 55g で変化なし。棄却。
3. **headerless 同期による時間ずれ**: 隣接コールバック間は ~2ms、関節角変化 ~0.0002 rad。
   重力方向の変化は無視できるレベル。90° の不整合は q=0 汚染で説明され、同期問題ではない。棄却。

### 未解決

55g ≈ 120g × 0.46 の系統的誤差。全仮説棄却後も原因未特定 (I-5)。

## 2026-03-15: I-4 修正後の実機結果

recording callback 内で kinematics を直接計算する方式に変更後の結果:

- フレーム数: 69 → **1253** (同期問題解消)
- OLS raw: 53g (以前の 55g とほぼ同じ — データ整合性改善では質量推定は変わらず)
- OLS+bias: **249g** (以前のデータでは -29g — フレーム数増加でバイアス推定が変化)
- 推定バイアス: Fx=-1.26, Fy=-1.24, Fz=-1.39 N (等方的、|bias|≈2.24 N)

OLS+bias の 249g は 120g の約 2 倍。バイアスと質量の分離が不十分な可能性あり。
OLS raw の 53g は変わらず、I-5 は未解決。

## 2026-03-16: I-5 根本原因の特定 — `/wrench` データソース問題

### LS プロセスの検証

合成データ (10 パラメータ線形回帰) で OLS (`np.linalg.lstsq`) と TLS (`solve_tls_weighted`) を検証。
ノイズなしで機械精度復元、ノイズありでもフレーム数増加で精度向上。全 10 テスト PASS。
→ ソルバー自体は健全。問題はインプット側にある。

### リグレッサ・wrench・キネマティクスの並列調査

3 サブエージェントで同時調査した結果:
- **リグレッサ構築**: Kubus et al. 2008 Eq.5-6 と一致。Lynch & Park 版との差 < 5.55e-17。問題なし
- **wrench 処理**: 座標系・符号・順序すべて正しい。問題なし
- **キネマティクス**: Pinocchio ベースの主実装は正しい（proper acceleration, LOCAL frame）。問題なし

### `/wrench` = `actual_TCP_force` の発見

UR ROS ドライバ `hardware_interface.cpp:534` を確認:
```cpp
readData(data_pkg, "actual_TCP_force", fts_measurements_);
```
RTDE ドキュメントより: `actual_TCP_force` = 「ペイロード補償済み。ゼロ化影響あり」

### UR ブレーキリリース時の自動ゼロ化を実証

スクリプト未起動状態で `/wrench` を監視:
- 起動時姿勢: 全成分 ≈ 0（グリッパ 960g が見えない）
- 姿勢変更後: Fy ≈ 11N, Fz ≈ -10N（重力方向変化分のみ出現）
- UR フォーラムで確認: 「When UR is powered on, the sensor is also reset」

### `ft_raw_wrench` の実機確認

`check_ft_raw.py` で RTDE 経由の `ft_raw_wrench` を 4 姿勢で計測:
- 構造的プリロード: Fz ≈ 25400N（センサ締結力）
- 重力射影ゼロの 2 姿勢 (フランジ Z = ベース X, Y): Fz ≈ 25396-25397（一致）
- フランジ Z = ±ベース Z: Fz ≈ 25408 / 25385（±11.5N の対称変動 = 重力パターン）
- `ft_raw_wrench` は起動時ゼロ化の影響を受けていない

### I-5 の根本原因

1. `actual_TCP_force` の起動時ゼロ化で F/T の絶対値が失われる
2. バイアス列 (6 列) で定数オフセットを吸収しようとしても、質量列の重力定常成分と共線性が発生
3. OLS raw (53g): ゼロ化で重力成分が消え、動的成分のみから推定 → 過小推定
4. OLS+bias (249g): バイアスと質量の分離不良 → 過大推定

### 次のステップ

`ft_raw_wrench` への移行 + プリロードキャリブレーション（6 面体姿勢計測）を計画。
計画ファイル: `.claude/plans/radiant-roaming-moon.md`

---

## 2026-03-15: I-1 修正

`replay_excitation_trajectory.py` の全 F/T 再ゼロ化箇所を除去。

- Phase 1: `controller.deactivate()` → teleop pub 停止 + `arm.activate_joint_trajectory_controller()` に置換
- Phase 3: `robot.deactivate_compliance()` → `arm.activate_joint_trajectory_controller()` に置換
- Phase 3: 明示的 `robot.zero_ft_sensor()` + sleep を削除
- osx_bilateral 側のコードは変更なし（`_arm` / `_teleop_active_pub` への直接アクセスで回避）
