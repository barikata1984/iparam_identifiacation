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
- `dynamics_utils.py` (Lynch & Park "bullet" 演算子ベース、現在は削除済み) の正しいリグレッサとの比較
- テストケース omega=[1,0,0], alpha=[0,0,0] で -1 が Izx (col 9) にあるべきところ Iyz (col 8) に出現

バグは `wrist_end_kinematics_utils.py` で修正済み（`0671248`）。`dynamics_utils.py` は 2026-03-24 のリファクタで削除。

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

## 2026-03-17: skip_teleop モード実装と実機計測

### スクリプト改修

`replay_excitation_trajectory.py` に `skip_teleop` モードを追加:
- `skip_teleop:=true`: `CompliantController(gripper_type=None)` で直接初期化（テレオペ/グリッパ不要）
- Phase 1 (teleop) / Phase 2 (gripper) / Phase 6 (resync) をスキップ
- launch ファイルに `skip_teleop` arg 追加、Dynamixel/haptic ノードを `unless` で抑制

マウント姿勢フローを追加:
- Phase 3a: フランジ上向き姿勢 `[90, -90, 90, 90, -90, 0]` deg（J4 反転）に移動
  → `zero_ftsensor` (bare flange) → wrench 表示 → グリッパ装着待ち → 装着後 wrench 表示
- Phase 3b: 励起軌道開始姿勢 `[90, -90, 90, -90, -90, 0]` deg に移動 → wrench 表示

### 実機計測結果

#### Bare flange 同定（23:19:13）

| パラメータ | OLS+bias | OLS raw |
|-----------|---------|---------|
| m [kg] | 0.012 | -0.006 |
| mcx | 0.008 | 0.007 |
| mcy | 0.085 | 0.027 |
| mcz | 0.035 | 0.040 |

期待通り全パラメータがゼロ近辺。パイプラインの動作確認として妥当。

#### グリッパ付き同定（23:45:39） — ゼロ化手順に問題あり

local→remote 切替時にプログラム再起動 → グリッパ装着状態で自動ゼロ化が発生。
OLS raw: m = 0.025 kg（期待 ~1 kg）。起動時ゼロ化がグリッパ荷重を吸収（I-6 の再現）。
OLS+bias: m = 0.980 kg, Fz bias = -9.35 N ≈ mg。バイアス列が重力を吸収。

#### グリッパ付き同定（18:07:47） — マウント姿勢フロー使用

マウント姿勢で zero_ftsensor (bare flange) → グリッパ装着 → ホーム移動。
local/remote 切替なし。

OLS raw: m = 0.298 kg（改善したが不十分）。
OLS+bias: m = 0.959 kg。
残差分析: Fx mean=+1.0, Fy mean=+1.0, Tx mean=-1.4 Nm の定数オフセットが残留。

### ft_raw_wrench 単位の特定（I-7 解決）

`check_ft_diff.py` で bare→loaded の差分を `actual_TCP_force` と `ft_raw_wrench` で同時計測:

| 計測 | actual_TCP_force Fz diff [N] | ft_raw_wrench Fz diff | ratio |
|------|------------------------------|----------------------|-------|
| Run 1 | -8.79 | -8.79 | 1.0000 |
| Run 2 | -10.66 | -10.66 | 1.0000 |
| Run 3 | -8.97 | -8.97 | 1.0000 |

**ft_raw_wrench の感度は 1 raw unit = 1 N**（全 3 回で一貫）。
巨大なオフセット（Fz ≈ 25,400）は構造的プリロードだが、変化量はそのままニュートン。

### F/T センサの短期揺らぎ発見（I-9）

同一姿勢・同一ペイロードで 10 分間隔の差分計測にて ±5 N の変動を観測。
Run 1, 3 は整合（-8.79, -8.97）、Run 2 は外れ値（-10.66）。
bare 読み取り時に ft_raw_wrench Fz が +4 N 跳ねた後、loaded 読み取りまでに戻った。
短期揺らぎは差分計測を汚染し、プリロードキャリブレーション精度に影響する。

### 主要発見のまとめ

1. `actual_TCP_force` の起動時ゼロ化は Polyscope 設定（TCP/ペイロード全ゼロ）でも回避不可
2. UR プログラム再起動（local→remote 含む）で自動ゼロ化が発生
3. `actual_TCP_force` と `ft_raw_wrench` の感度は 1:1（ゲイン誤差なし）
4. F/T センサに ±5 N の短期揺らぎあり（I-8 の長期ドリフトとは別）
5. ボルト締結プリロード仮説は棄却（Tz に影響なし + ボルトプリロードは内力ループ）

### 作成ファイル

- `scripts/check_ft_diff.py`: bare/loaded の actual_TCP_force + ft_raw_wrench 差分計測ツール

### 追加計測: ロボット再起動後の差分計測（2 回）

再起動後 10 分インターバルで計測:

| Run | Fz diff [N] | m_est [kg] | bare raw Fz | 備考 |
|-----|------------|------------|-------------|------|
| 5 | -10.20 | 1.040 | 25388.89 | 再起動直後。bare actual_TCP_force ≈ 0（起動時ゼロ化の直接証拠） |
| 6 | -11.74 | 1.197 | 25395.86 | 10 分後。bare raw が +7.0 ドリフト |

再起動直後は ft_raw_wrench ドリフトが特に大きい（10 分で +7 N）。
全 6 回の推定範囲: 0.90〜1.35 kg（秤 0.956 kg に対して ±20%）。

### ft_raw_wrench Fz 感度の追加確認

Fz 以外の軸（Fx, Fy）は差分が小さすぎて検証不十分。
トルク軸（Tx, Ty, Tz）も同様に未検証。
厳密には「Fz 軸の感度 = 1 N/unit」のみが確認済み。

### UR フォーラム情報: F/T センサドリフトは仕様

出典: https://forum.universal-robots.com/t/drift-in-the-ur3e-f-t-sensor/41493

- UR の F/T センサ（Robotiq 製）は**連続的な測定精度ではなく、定期的なゼロ校正を前提に設計**
- 各計測前に `zero_ftsensor()` で再ゼロ化して使うことが推奨
- ドリフトの一貫性は保証されていない（設計仕様）
- 力制御ノード（"Force-based move", "Tool Contact"）は内部で自動ゼロ校正を実施

慣性パラメータ同定にとって厄介: `zero_ftsensor()` するとペイロードの重力成分もゼロ化される。
対策候補:
1. 軌道直前に `zero_ftsensor()` + リグレッサに定数列追加（共線性リスクあり）
2. ft_raw_wrench + 軌道直前のプリロード計測で差し引く
3. 励起軌道を短く保ち（5 秒）、ドリフトの影響を最小化

### 外付け F/T センサの調査

UR 内蔵 F/T センサの精度限界（±3.5 N precision、定期的 zero_ftsensor 前提の設計）を踏まえ、
外付けセンサの選択肢を調査。

| | UR 内蔵 | ATI 等外付け |
|---|---------|------------|
| 力精度 | ±4 N | ±0.1〜0.5 N |
| 力 precision | ±3.5 N | 桁違いに高い |
| ドリフト | 大（±5 N 短期揺らぎ確認済み） | 温度補償あり、桁違いに小さい |
| 設計思想 | 定期的 zero_ftsensor 前提 | 連続計測精度 |

主な UR5e 対応製品:
- **ATI Axia80**: URCap プラグインで UR コントローラに直接統合。慣性パラメータ同定論文で最多使用
- **ATI Mini45 / Gamma**: 研究用途で広く使用。シリコンストレインゲージで高分解能・高剛性
- **Bota Systems SensONE**: UR CB/E-Series 向けプラグ＆プレイキット

120g 物体の同定には mg ≈ 1.2 N の信号検出が必要。内蔵センサの precision ±3.5 N では原理的に困難。
外付けセンサによりドリフト・ゼロ化問題を大幅に軽減可能。

参考:
- https://www.ati-ia.com/products/ft/sensors.aspx
- https://www.universal-robots.com/blog/universal-robots-with-ati-s-ft-sensors-just-feels-right/
- https://www.botasys.com/robot-accessories/collaborative-robot-ft-sensor-kit

### 次のステップ

- I-9 の特性評価（揺らぎの時間スケール・分布の把握）
- ft_raw_wrench ベースの同定への移行（Step 5d）: プリロードを軌道直前に計測し差し引く方式
- 外付け F/T センサの導入検討（精度要件に応じて）

## 2026-03-25: グリッパキャリブレーション・差分法・物体同定

### グリッパキャリブレーション（Step 6）

skip_teleop モード 5 試行（開始姿勢でゼロ化、グリッパのみ、秤実測 ≈ 950g）:

| 手法 | 質量平均 | 誤差率 | CV% |
|---|---|---|---|
| OLS+bias | 1.010 kg | +6.3% | 0.65% |
| TLS+bias (Partial EIV) | 1.357 kg | +42.8% | 0.72% |

OLS+bias が最良。4 手法の平均を `data/calibration/gripper.json` に保存。

### 差分法による物体同定（Step 7-8）

teleop モードで 120g スポンジブロックを把持し 5 試行。差分法 φ_object = φ_total - φ_gripper:

| 手法 | 質量平均 | 誤差 | CV% |
|---|---|---|---|
| OLS+bias | 184g | +64g (+53%) | 3.4% |
| TLS+bias | 193g | +73g (+61%) | 5.2% |
| OLS (raw) | 40g | -80g (-67%) | 27.2% |
| TLS (raw) | 48g | -72g (-60%) | 29.2% |

再現性は良好（OLS+bias CV 3.4%）だが、+53% の系統的過大推定が残存。

### 過大推定の原因分析

φ_gripper は skip_teleop（**開始姿勢**でゼロ化）、φ_total は teleop（**ホーム姿勢**でゼロ化）
で取得。ホーム姿勢と開始姿勢は J6 のみ 90° 異なる（他の 5 軸は同一）。
J6 はツール Z 軸回転なので重力射影は同一のはず。

しかし 64g × 9.81 ≈ 0.63 N の系統的差異があり、これはゼロ化タイミングの
F/T センサドリフト（I-8, I-9）で説明可能な範囲。

### 次ステップ

ホーム姿勢を開始姿勢に揃え、同一セッション内で φ_gripper → φ_total を連続計測して
ゼロ化条件を完全に一致させる。
