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

## 2026-03-16: Step 5a/5b — プリロードキャリブレーション基盤の構築

ft_raw_wrench 移行の準備として、6面体姿勢定義・計測スクリプト・RViz プレビューを実装。

### 作成ファイル

1. **`src/calibration/cube_poses.py`** — 6面体姿勢定義モジュール
   - `compute_cube_poses()`: Pinocchio CLIK (damped least-squares IK) で6姿勢を算出
   - 基準姿勢 `[90, -90, 90, -90, -90, 0]` deg (フランジ Z = -Z) と同一TCP位置で6方向
   - `CubePose` dataclass (label, flange_z_direction, joint_angles_rad)

2. **`scripts/calibrate_ft_preload.py`** — 実機計測スクリプト (ur_rtde 直接、ROS不要)
   - 各姿勢: ユーザ確認 → moveJ (0.5 rad/s) → 2s 静止 → 3s サンプリング (500Hz)
   - 統計サマリ: プリロード推定値、姿勢間σ、最大偏差、ノイズσ
   - JSON 保存: `results/ft_preload_calibration_<timestamp>.json`

3. **`scripts/preview_cube_poses.py`** — RViz プレビュー (ROS ノード)
   - 既存 `preview_trajectory.launch` を活用 (`/preview/joint_states` に publish)
   - Enter で次姿勢、1-6 でジャンプ、a で自動サイクル (3s/pose)

4. **`tests/test_cube_poses.py`** — 16 テスト全 PASS
   - 各姿勢の FK 検証: フランジ Z が期待方向と一致 (atol=1e-4)
   - TCP 位置が基準姿勢と一致 (atol=1mm)

### 変更ファイル

- `setup.py`: `calibration` パッケージ追加
- `src/calibration/__init__.py`: 新規 (空)

### 次のステップ

5c: 実機で `calibrate_ft_preload.py` を実行し、プリロードの姿勢不変性を検証。

## 2026-03-16: Step 5c — プリロードキャリブレーション実機計測と ft_raw_wrench 調査

### 実施内容

1. RViz プレビューに septic spline 遷移シミュレーションを追加（`preview_cube_poses.py` に `s` コマンド）
2. `src/calibration/septic_spline.py` を新規作成（7次多項式 rest-to-rest 補間）
3. `calibrate_ft_preload.py` を拡張し `actual_TCP_force` も同時収集
4. bare flange で 3 回計測（18:42, 21:40/21:46, 22:05）

### 計測結果: ft_raw_wrench preload の時間ドリフト

| 成分 | 18:42 | 21:40 | 21:46 | 22:05 |
|------|------:|------:|------:|------:|
| Fx | 1806.56 | 1794.09 | 1794.11 | 1793.85 |
| Fy | 419.80 | 392.91 | 392.94 | 392.45 |
| Fz | 25382.72 | 25406.10 | 25405.98 | 25406.82 |
| Tx | -7.365 | -7.888 | -7.882 | -7.890 |
| Ty | -17.519 | -17.525 | -17.533 | -17.530 |
| Tz | -76.733 | -75.964 | -76.037 | -75.914 |

- 3 時間で Fy: -27, Fz: +23 のドリフト（温度依存と推定）
- 6 分以内は安定（差分 < 0.1）
- 19 分で Fy: -0.5 程度のドリフト
- 19 分ドリフト × 9 ≈ 3 時間と仮定すると実際の 3 時間ドリフトの 1/3〜1/6 → 22 時時点でもまだ収束していない

### 計測結果: actual_TCP_force（ペイロード 0 lbs 設定）

- 全成分がほぼゼロ（mean < 0.3）→ N/Nm 単位の傍証
- ただし姿勢間で最大 1.2 の偏差あり（15 分後は悪化）
- `actual_TCP_force` もドリフトする

### 各計測内の姿勢間ばらつき

- ft_raw_wrench 姿勢間 σ ≈ 1.6（Fx, Fz）、ノイズ σ ≈ 0.1
- 姿勢間ばらつきの意味: ft_raw_wrench はセンサフレームで出力されるため、異なる姿勢での値は異なる物理方向を指す → 単純なスカラー比較は不適切

### ft_raw_wrench の公式定義（UR RTDE Guide）

> "Raw force and torque measurement given in the Tool Flange frame. Not compensated for forces and torques caused by the payload. Not zeroed by zero_ftsensor()."

- 座標系: Tool Flange frame（センサフレーム）
- ペイロード補償なし、`zero_ftsensor()` の影響なし
- **単位の明記なし**（UR 公式ドキュメントのどこにも記載がない）
- Version 5.9.0 で追加

### ft_raw_wrench の単位問題

- Fz ≈ 25,406 がニュートンなら 2.5 トンに相当 → bare flange でありえない → **そのままニュートンではない**
- Fx ≈ 1794, Fy ≈ 393, Fz ≈ 25406 と各軸で桁が大きく異なる → ストレインゲージのブリッジオフセット（ハードウェアバイアス）が支配的
- 単位特定のために base フレームへの変換を試みたが、巨大なバイアスが回転して散るため有効な情報が得られず
- 線形回帰（bias + α·c(i) モデル）では換算係数と質量が分離できず、ニュートン単位のゼロ補正には直結しない
- **既知質量での実測が換算係数特定の最も確実な方法**

### UR 内部実装の調査

- `zero_ftsensor()`: 呼出時の観測値を定数オフセットとして記憶し減算（URScript マニュアル）
- `set_target_payload(m, cog)`: 「The internal force/torque sensor in the robot tool is reset each time the payload is updated. This is similar to the behaviour of zero_ftsensor().」（URScript マニュアル 5.19）
- 起動時の自動ゼロ化はペイロード設定の適用に付随して起こる可能性が高い
- ペイロード補償が単純オフセットか姿勢依存の動的補償かは不明（UR コントローラファームウェアが非公開）
- ur_rtde の `setPayload()`, `zeroFtSensor()` はコマンドを送るだけで、補償ロジックは UR コントローラ内部で処理

### 未解決の問題

1. ft_raw_wrench の単位が不明（ドキュメントに記載なし）
2. 換算係数の特定方法が未確立（既知質量での実測が必要）
3. 時間ドリフトがあり、プリロードを定数として扱うアプローチに限界がある
4. 信号と力の線形関係が未検証

### 参照先

- UR RTDE Guide: `docs.universal-robots.com/tutorials/.../rtde-guide.html`
- ur_rtde API: `sdurobotics.gitlab.io/ur_rtde/api/api.html`
- URScript Manual 5.19 set_target_payload: `universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-scriptmanual/G5/set_target_payload.htm`
- ur_rtde ソース: `gitlab.com/sdurobotics/ur_rtde/-/raw/master/src/rtde_control_interface.cpp`
- 計測結果: `results/ft_preload_calibration_2026-03-16_*.json` (3 ファイル)

---

## 2026-03-15: I-1 修正

`replay_excitation_trajectory.py` の全 F/T 再ゼロ化箇所を除去。

- Phase 1: `controller.deactivate()` → teleop pub 停止 + `arm.activate_joint_trajectory_controller()` に置換
- Phase 3: `robot.deactivate_compliance()` → `arm.activate_joint_trajectory_controller()` に置換
- Phase 3: 明示的 `robot.zero_ft_sensor()` + sleep を削除
- osx_bilateral 側のコードは変更なし（`_arm` / `_teleop_active_pub` への直接アクセスで回避）
