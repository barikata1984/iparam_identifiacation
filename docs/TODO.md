# TODO — マイクロタスク

> **最終更新**: 2026-03-17

---

## 推定精度修正

- [x] Step 1: F/T 再ゼロ化を全箇所で除去 (I-1) — `daa2939`
- [x] Step 2: リグレッサ行列 Row 5, 6 を修正 (I-2) — `0671248`
  - `dynamics_utils.py` と 10 テストケースで一致確認
- [x] Step 3: 55g 推定の原因特定 (I-5) → **根本原因: `/wrench` = `actual_TCP_force` の起動時ゼロ化 (I-6)**
  - LS プロセス自体は合成データで検証済み（問題なし）
  - リグレッサ・wrench・キネマティクスも個別に正しいことを確認
  - `actual_TCP_force` のゼロ化オフセットとバイアス列の共線性が推定を破壊
- [x] Step 4: q=0 汚染の修正 (I-4) — recording 内直接計算で解消済み
- [ ] Step 5: `ft_raw_wrench` への移行 (I-6)
  - [x] 5a: プリロードキャリブレーション（6 面体姿勢で無負荷 ft_raw_wrench を計測）
  - [x] 5b: RViz で 6 姿勢プレビュー
  - [x] 5c: 実機でプリロード計測（3 回実施、時間ドリフトと単位問題を発見）
  - [x] 5c': ft_raw_wrench の単位を特定する → **感度 1 raw unit = 1 N を実測確認** (I-7 解決)
  - [ ] 5c'': 時間ドリフトの収束条件を確認する（暖機時間の特定）
  - [ ] 5c''': F/T センサの短期揺らぎ (±5 N) の特性評価
  - [ ] 5d: `replay_excitation_trajectory.py` を ft_raw_wrench 対応に改修
- [x] Step 6 (予備): ペイロードなし bare flange で skip_teleop モードの動作確認
  - m ≈ 0.012 kg (OLS+bias), -0.006 kg (OLS raw) — 期待通りゼロ近辺
- [ ] Step 6: グリッパキャリブレーション（物体なしで励起軌道実行 → φ_gripper 推定）
  - グリッパ付き実機計測を 2 回実施（actual_TCP_force 起動時ゼロ化問題で推定精度不十分）
  - skip_teleop モードにマウント姿勢フロー追加済み（zero_ftsensor → グリッパ装着 → ホーム移動）
- [ ] Step 7: 物体同定（差分法: φ_object = φ_total - φ_gripper）
- [ ] Step 8: 120g 物体に対して正常な推定値を確認

---

## TLS スケーリング改善（I-3）

- [ ] `ScalingMode.NOISE_BASED` を `tls.py` に実装（diff ベースの T + D=I）
- [ ] 既存の 3 モードと並列比較実験（OLS との乖離が改善されるか検証）
- [ ] 効果確認後、再帰的 TLS にもスケーリングを組み込む
