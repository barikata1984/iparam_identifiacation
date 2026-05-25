# Log: F/T センサ挙動と妥当性

UR5e 内蔵 F/T センサ (`/wrench` = RTDE `actual_TCP_force`) の挙動・単位・確度の調査記録。
関連既存ログ: `log_estimation_accuracy.md` (再ゼロ化バイアス), `plan_ft_preload_calibration.md` (ft_raw プリロード), `wrench_topic_and_coordinate_frames.md` (トピック/座標系).

## 2026-05-24: 起動時オートゼロ・単位・確度の実機確証

### 発見1: `actual_TCP_force` は起動 (ブレーキリリース) 時に「その姿勢で」自動ゼロ化される

`plan_ft_preload_calibration.md` で「ブレーキリリース時に自動ゼロ化」と既述だが, 今回**ゼロ化が起動時点の姿勢を基準に行われる**ことを実機で確証した。

検証 (グリッパ+カップリング装着, payload はペンダントで 0):
- **down 姿勢で起動** → down で Fz≈+0.9 (≈0), up で Fz≈−16.7
- **up 姿勢で起動** → up で Fz≈+0.4 (≈0), down で Fz≈+19.3

→ ゼロになる姿勢が起動姿勢に追従。すなわち

```
Fz_meas(現在姿勢) = g_proj(現在姿勢) − g_proj(起動姿勢) + 微小バイアス
```

第 2 項 (定数) = −(起動姿勢の重力射影)。起動姿勢がフランジ鉛直なら大きさ ~mg (~9 N)。
起動姿勢から重力に対し反転した姿勢では, 実重力 (+mg) と焼き込みオフセット (+mg) が同符号で足さり **+2mg** に見える (実測 down=+19.3 ≈ 2mg)。

ドライバ/URScript 側の関与は否定済み: `external_control.urscript` の `zero_ftsensor()`/`set_payload()` はコマンド受信時のみ, `hardware_interface.cpp` もサービス経由のみ。`set_payload()` はリポジトリ内で**呼び出しゼロ** (定義のみ)。よってこれは UR コントローラ本体の起動時挙動。

### 発見2: 定数オフセットの扱い — bias 項 / 同一姿勢差分

起動オートゼロによる定数オフセットは**セッション内で不変**なので,

- 同定の **bias 付き手法 (OLS+bias / TLS+bias) が吸収**する。
- あるいは**同一姿勢で bare と loaded を差分**すれば, オフセットも定常バイアスも相殺され, 追加重量そのものが得られる (差分法 / preload 校正の原理)。バイアス値を知らなくてよい点でロバスト。

### 発見3: 力スケールの妥当性検証 (合格)

同一 up 姿勢で bare vs loaded:
- bare @ up: Fz≈0, loaded @ up: Fz=−8.88 N → **ΔFz=8.88 N**
- `m_meas = 8.88/9.81 = 0.905 kg` (≈905 g) vs 実測 ~950 g → **~5% 低** (姿勢の非鉛直 cos 誤差・概算質量・センサ確度で説明可)。

→ **`/wrench` の力は N 単位で信頼できる** (質量レベルで ~5%)。「読みが N で間違っている」最悪ケースは否定。

### 発見4: `ft_raw_wrench` は未較正の生単位 (N ではない)

RTDE `getFtRawWrench()` の値は Fz≈25318 (期待 ~24800, `preview_cube_poses.py` のダミー値と一致) で, **N ではなく未較正の生カウント**。`plan_ft_preload_calibration.md` の「~25000N」は単位的に誤り (訂正済み: 生単位)。
→ 生単位ゆえ SI 慣性同定に直接使えない (軸ごと未知ゲインが必要)。`actual_TCP_force` (較正済み N) が上位互換。
→ 生値を ROS で取る試み (driver recipe に `ft_raw_wrench` 追加 + python `ur_rtde` 直叩き) は, ur_rtde 未インストール・robot が当該フィールド未提供で頓挫。**この路線は破棄** (ドライバ改変は stash, replay の ur_rtde コードは撤去済み)。

### 発見5: センサ確度の床 — 慣性テンソルは不可観測

UR5e 内蔵 F/T 仕様: 力 精度 ±3.5 N / 確度 ±4 N, トルク 精度 ±0.2 / 確度 ±0.3 Nm。
~0.95 kg 級ペイロードの信号と比較:

| 推定対象 | 信号 | 確度床 | 可否 |
|---|---|---|---|
| 質量 | 重力 ~9 N | ±4 N | △〜○ (多姿勢平均で ~5%) |
| 重心 (CoM) | 重力トルク ~0.3–0.5 Nm | ±0.3 Nm | △ (床ぎりぎり) |
| 慣性テンソル | I·α ~0.005–0.05 Nm | ±0.3 Nm | ✗ (床より 1–2 桁小, 完全に埋もれる) |

→ 読みが正しくても**信号が分解能を下回るため, アルゴリズムでは回復不能**。慣性テンソルが必要なら外付け高精度 F/T or 重い/激しいテスト体が必須。

### 発見6: Tz の段差 = wrist_3 のクーロン摩擦混入

`actual_TCP_force` の Tz に現れる bang-bang 的段差は, **wrist_3 関節の乾性摩擦が漏れ込んだもの**。
- corr(Tz_lowpass, −sign(wrist_3 速度)) ≈ 0.93–0.99, 振幅 ~0.15 Nm (姿勢一致対で重力相殺後も再現)。
- エンコーダラップでも tare アーティファクトでも量子化でもない (生 wrench, 連続値, 500 Hz 重複なし)。
- 反転時に ~200 Hz 構造共振バーストを伴う。
- 静止計測 (速度ゼロ) では出ないので, 妥当性検証は静的姿勢で行うべき。

### 裏付け資料 (リプレイレコーディング)

`results/replay_recording_2026-05-24_21-35-06` を `assets/2026-05-24_replay_recording/` にコピー:
- `replay_recording.png`: F/T・tip 位置/速度/加速度の総合プロット (起動オートゼロによる Fz オフセットが見える)。
- `wrist3_position_torque_z.png`: wrist_3 関節位置 vs Tz — 発見6 (wrist_3 摩擦) の根拠。Tz の段差が wrist_3 の turning point と一致。
- `velocity_acceleration_comparison.png`: 数値微分 vs ヤコビアン (参考)。
- `recording.npz`: 生データ (time, wrench, joint_position/velocity, tip_*)。`ft_raw_wrench` は当該 run では NaN (ur_rtde 未導入のため; 発見4 参照)。

### 結論・方針

- 同定は **`/wrench` (actual_TCP_force, payload=0) + bias 付き手法**で行う。ft_raw・payload 操作は不要。
- 力/質量は信頼可 (~5%)。**CoM は際どく, 慣性テンソルは内蔵 F/T では不可観測** — ハード更新 (外付け F/T) の判断材料。
- 励起軌道は gravity に対する tool 姿勢を十分振ること (mass/CoM の可観測性確保, wrist_3 主体は不可)。
