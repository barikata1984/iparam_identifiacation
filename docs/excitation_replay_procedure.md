# 励起軌道リプレイによる慣性パラメータ同定 — 実行手順

## 前提条件

- UR5e ドライバが起動済み（F/T センサのゼロ化を含む、**物体なし**の状態で実施すること）

```bash
roslaunch osx_ur5e connect_real_robot.launch
```

## 実行

```bash
roslaunch iparam_identification replay_excitation_trajectory.launch \
    trajectory:=$(rospack find iparam_identification)/data/trajectories/excitation_trajectory.json
```

### 主要引数

| 引数 | デフォルト | 説明 |
|------|-----------|------|
| `trajectory` | `data/trajectories/excitation_trajectory.json` | 励起軌道 JSON |
| `wrench_topic` | `/wrench` | F/T センサトピック |
| `trim_start` | `1.0` | 同定に使用する開始時刻 [s] |
| `trim_end` | `4.0` | 同定に使用する終了時刻 [s] |
| `leader_port` | `/dev/ttyUSB0` | Dynamixel リーダーアームのポート |

## 対話フロー

1. **Phase 1 — 遠隔操作把持**: ホーム位置へ移動 → リーダーアーム同期 → 遠隔操作で物体を把持 → **Enter** で次へ
2. **Phase 2 — グリッパ全閉**: 自動実行
3. **Phase 3 — 初期姿勢移動**: 励起軌道の開始関節角度を表示 → **Enter** で移動
4. **Phase 4 — 軌道リプレイ + 記録**: **Enter** でリプレイ開始（自動記録）
5. **Phase 5 — 慣性パラメータ同定**: OLS/TLS 結果を表示 → `y`/`n` で publish 判断
6. **Phase 6 — リーダー再同期**: ホーム復帰 → リーダーアーム再同期

## 結果

`results/excitation_replay_<timestamp>/` に保存:
- `result.json` — 推定パラメータ、使用フレーム、メタ情報
- `velocity.png`, `acceleration.png`, `wrench.png` — 記録データのプロット

## 既知の問題

→ `docs/ISSUES.md` を参照
