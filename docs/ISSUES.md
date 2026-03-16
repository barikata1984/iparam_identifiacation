# ISSUES — 横断的課題

> **最終更新**: 2026-03-15
>
> 解決した課題は項目ごと削除する。

---

## I-1: 励起軌道リプレイ中の F/T センサ再ゼロ化

**発見**: 2026-03-15

`replay_excitation_trajectory.py` Phase 3 で物体把持後に F/T センサが再ゼロ化される。

- `deactivate_compliance()` (line 232) が内部で `zero_ft_sensor()` を呼び出す
- 明示的 `zero_ft_sensor()` (line 248) でも再ゼロ化
- UR の `zero_ftsensor` は呼出時の読み値を定数オフセットとして差し引くため、物体重量分のバイアスが全測定から除去される
- リグレッサは `F = m * a_proper` を仮定するが、測定値は `F_true(q) - F_true(q0)` となり定数バイアスが発生
- 120g 物体でバイアス ~1.18 N + グリッパ重量 — ペイロード信号 (~1.18 N) と同等以上
- 結果: 質量推定が 180g / 10g / 負値など異常値になる

**対応**: Phase 3 の `deactivate_compliance()` と `zero_ft_sensor()` を除去し、ドライバ起動時のゼロ点を維持 → TODO 参照

---

## I-2: リグレッサ行列 Row 5, 6 の交差慣性項バグ

**発見**: 2026-03-15

`wrist_end_kinematics_utils.py` の `get_regressor_matrix()` で Row 5 (Ny) と Row 6 (Nz) の交差慣性項に入れ替えバグがある。Row 4 (Nx) は正しい。

- Row 5: Iyy <-> Ixy スワップ、Iyz <-> Izx スワップ
- Row 6: Ixy/Iyz/Izx の循環置換 + Ixy 係数の変数誤り (wx^2 -> wz^2)
- `dynamics_utils.py` に Lynch & Park ベースの正しい実装が存在するが、`tool0_kinematics_node.py` はバグ版を使用

**影響**: 慣性テンソル推定が直接狂い、最小二乗のカップリングで質量・重心にも波及（I-1 より影響は軽微）

**対応**: `dynamics_utils.py` の正しい実装を参照して Row 5, 6 を修正 → TODO 参照

---

## I-3: TLS スケーリング行列がデータ分散ベース

**発見**: 2026-03-12

`tls.py` の `COLUMN_ONLY` モードで `T = diag(1/std(列))` を使用。std はデータのばらつき（信号+ノイズ）であり、ノイズレベルではない。高加振列（良い SNR）の信頼度が不当に低く扱われる。

**対応**: `ScalingMode.NOISE_BASED` を追加（diff ベースのノイズ推定）→ TODO 参照
