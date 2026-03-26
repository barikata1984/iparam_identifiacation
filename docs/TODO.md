# TODO — マイクロタスク

> **最終更新**: 2026-03-25

---

## 推定精度修正

- [x] Step 1: F/T 再ゼロ化を全箇所で除去 (I-1) — `daa2939`
- [x] Step 2: リグレッサ行列 Row 5, 6 を修正 (I-2) — `0671248`
  - 8 テストケースで構造検証 + 既知値テスト 2 件で確認
- [x] Step 3: 55g 推定の原因特定 (I-5) → **根本原因: `/wrench` = `actual_TCP_force` の起動時ゼロ化 (I-6)**
  - LS プロセス自体は合成データで検証済み（問題なし）
  - リグレッサ・wrench・キネマティクスも個別に正しいことを確認
  - `actual_TCP_force` のゼロ化オフセットとバイアス列の共線性が推定を破壊
- [x] Step 4: q=0 汚染の修正 (I-4) — recording 内直接計算で解消済み
- [x] Step 5: `ft_raw_wrench` 調査 (I-6) — バイアス付き推定で十分と判明、移行不要
  - [x] 5a: プリロードキャリブレーション（6 面体姿勢で無負荷 ft_raw_wrench を計測）
  - [x] 5b: RViz で 6 姿勢プレビュー
  - [x] 5c: 実機でプリロード計測（3 回実施、時間ドリフトと単位問題を発見）
  - [x] 5c': ft_raw_wrench の単位を特定する → **感度 1 raw unit = 1 N を実測確認** (I-7 解決)
- [x] Step 6 (予備): ペイロードなし bare flange で skip_teleop モードの動作確認
  - m ≈ 0.012 kg (OLS+bias), -0.006 kg (OLS raw) — 期待通りゼロ近辺
- [ ] Step 6: グリッパキャリブレーション（物体なしで励起軌道実行 → φ_gripper 推定）
  - グリッパ付き実機計測を 2 回実施（actual_TCP_force 起動時ゼロ化問題で推定精度不十分）
  - skip_teleop モードを簡素化済み（Phase 3a マウント姿勢フロー削除、開始姿勢で zero_ftsensor）
  - 推定手法を 4 並列化済み（OLS, TLS, OLS+bias, TLS+bias）
  - Partial EIV 実装済み（バイアス列をエラーフリーに指定）
  - 実機 5 試行の結果: OLS+bias m≈1.01kg (+6%), TLS+bias m≈1.36kg (+43%) — **OLS+bias を主推定手法に**
- [ ] Step 7: 物体同定（差分法: φ_object = φ_total - φ_gripper）
- [ ] Step 8: 120g 物体に対して正常な推定値を確認

---

## TLS スケーリング改善（I-3）

- [x] 文献調査: WTLS 重み行列の理論的根拠（26論文、`docs/SURVEYS/wtls_scaling_matrix.md`）
- [x] `ScalingMode` を再設計: `IDENTITY` / `DATA_VARIANCE` / `NOISE_VARIANCE`
  - `NOISE_VARIANCE`: `t_i = 1/σ_noise_i`（diff ベース推定）— ML 最適（ガウスノイズ下）
  - `DATA_VARIANCE`: 旧 `COLUMN_ONLY` の改名（数値正規化、統計的根拠なし）
  - `FULL` を削除（行の std 正規化に理論的根拠なし）
  - デフォルトを `NOISE_VARIANCE` に変更
- [x] テスト追加: 全モードの無ノイズ復元テスト、ノイズ有りテスト（12/12 パス）
- [x] Partial EIV 実装: バイアス列をエラーフリーに指定（Van Huffel & Vandewalle 1989）
  - 射影法で A1 を除去 → 縮小 TLS → OLS で復元
  - テスト 3 件追加（合成データで Full TLS より Partial EIV が高精度を確認）
- [x] 実機 5 試行で検証: TLS+bias 質量 1.60→1.36 kg に改善（但し +43% の過大推定が残存）
  - 重力列とバイアス列の残留相関が原因。OLS+bias (1.01 kg, +6%) が現時点で最良
- [x] CLI 切替: `tls_scaling` ROS パラメータ追加（launch/rosrun で指定可能）
- [x] 結果出力に `tls_scaling` メタデータ追加
- [x] 同定ループ: n 選択時に再試行（start pose → replay → identify のループ）
- [ ] 効果確認後、再帰的 TLS にもスケーリングを組み込む
