# ISSUES — 横断的課題

> **最終更新**: 2026-05-24
>
> 解決した課題は項目ごと削除する。

---

## I-11: 内蔵 F/T センサの分解能が CoM/慣性同定に不足

UR5e 内蔵 F/T (`actual_TCP_force`) の確度は力 ±4 N / トルク ±0.3 Nm。
~0.95 kg 級ペイロードでは重力トルク ~0.3–0.5 Nm が確度床と同オーダー、慣性トルク
`I·α ~0.005–0.05 Nm` は床より 1–2 桁小さく**完全に埋もれる**。

- 帰結: 質量は推定可 (多姿勢平均で ~5%)、CoM は際どい、**慣性テンソルは不可観測**。
  読み値自体は N で妥当 (発見3) なので、アルゴリズム改良では回復不能。
- 改善案: (a) 外付け高精度 6 軸 F/T、(b) より重い/大きいテスト体で SNR 確保、
  (c) 励起の角加速度を上げる (wrist_3 摩擦・安全に注意)。
- 詳細: `docs/LOGS/log_ft_sensor_behavior.md`

## I-12: FT 300-S 使用時に OLS+bias total mass が物体質量にしかならない

パイプラインは合成データで完璧に動作(OLS+bias 誤差 0). 実データでのみ
total mass が ~0.34 kg(期待 ~1.27 kg = gripper + object). bias が
グリッパ重力を過剰に吸収している疑い.

- 条件数(17.1), 時間同期は問題なし
- 符号規約も start pose での静的比較で確認済み(反転不要)
- 詳細: `docs/LOGS/log_ft300s_integration.md`

## I-10: cmodel_urcap_driver の無音死を起動時に検出できない

`connect_real_robot.launch` 起動時に Polyscope 側 URCap が READY でないと
`cmodel_urcap_driver` が socket 例外で即死するが、`cmodel_action_controller` は
ActionServer を立ち上げるため上位コードからは「接続成功」に見える。
グリッパ指令を送っても物理動作せず、ログ上は `Gripper fully closed.` まで
進行してしまう。

- 即時回避: 起動順を守る（Polyscope で External Control を Play してから launch）
- 復旧: `rosrun robotiq_control cmodel_urcap_driver.py 192.168.55.20` を別ターミナルで起動
- 改善案: `replay_excitation_trajectory.py` 起動時に `/status` の Publisher 不在を
  検出してエラー終了する、または `wait_for_message('/status', timeout=2.0)` を
  `RobotiqGripperController.__init__` に追加する
- 詳細: `docs/LOGS/log_robotiq_driver_silent_death.md`
