# Pinocchio ベース batch_identifier 修正計画

## 背景

既存の `wrist_end_kinematics_node.py` (ur_pykdl ベース) では、慣性パラメータが期待値と異なる結果となっていた。新規実装した pinocchio ベースの `tool0_kinematics.py` を使用するように修正した。

## 実装状況

| Phase | 内容 | 状態 |
|-------|------|------|
| Phase 1 | tool0_kinematics.py の拡張 | ✅ 完了 |
| Phase 2 | tool0_kinematics_node.py の作成 | ✅ 完了 |
| Phase 3 | Launch ファイルの更新 | ✅ 完了 |
| Phase 4 | 検証 | ✅ 基本検証完了 |

## 重要な発見・知見

### 1. Wrench の符号について（重要）

**当初の誤り**: `batch_identifier.py` では wrench を反転していた（反力→作用力の変換のため）。

```python
# 誤り: 反転は不要だった
f = [-wrench_msg.wrench.force.x, ...]
```

**正しい処理**: UR の内蔵 F/T センサーは、**payload がセンサーに加える力**を直接報告する。反転は不要。

```python
# 正しい: そのまま使用
f = [wrench_msg.wrench.force.x, ...]
```

**検証方法**: 静止状態で以下を確認
- `la_tool0.z ≈ 9.8 m/s²`（tool0 が下向きの場合）
- `wrench.force.z ≈ m * 9.8 N`（正の値）
- 両者の符号が一致していれば正しい

### 2. F/T センサーのゼロ点補正

**重要**: ゼロ点補正のタイミングが推定精度に大きく影響する。

**自動ゼロ点補正**: `batch_identifier.py` は記録開始時（Enter 押下直後）に自動的にゼロ点補正を実行する。

```python
# run_cli() 内で自動実行
input("Press [Enter] to zero F/T sensor and START recording...")
self.zero_ft_sensor()  # 自動でゼロ点補正
```

**正しい運用手順**:
1. **物体を把持した状態**で姿勢を取る
2. Enter を押す → 自動でゼロ点補正が実行される
3. 記録が開始される
4. ロボットを動かしてデータ収集

**注意**: ゼロ点補正は「物体を把持した状態」で行われるため、推定される質量は「グリッパー込みの総質量」ではなく「把持した物体のみの質量」となる。グリッパーの質量も含めて推定したい場合は、別途手動でゼロ点補正のタイミングを調整する必要がある。

### 3. OLS vs TLS

**観察**: TLS（Total Least Squares）は異常に大きな値を出すことがある。

| 手法 | 370g 物体の推定結果 |
|------|---------------------|
| OLS | m ≈ 0.31 kg（妥当） |
| TLS | m ≈ 0.88 kg（異常） |

**結論**: 現状では **OLS を使用**する。TLS は今後の改善検討事項。

### 4. 慣性パラメータの順序

回帰行列のパラメータ順序は以下のとおり（論文とは異なる）:

```
[m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Iyz, Izx]
```

ここで:
- `h = m * c`（質量 × 重心位置）
- 慣性テンソルは対称なので `Izx = Ixz`

## 検証結果

### 静止状態テスト（370g 物体）

| 項目 | 期待値 | 実測値 | 誤差 |
|------|--------|--------|------|
| 質量 m | 0.370 kg | 0.307 kg (OLS) | -17% |
| Matrix rank | 10 | 10 | - |

誤差の原因として考えられるもの:
- F/T センサーのノイズ
- 数値微分によるフィルタ遅延
- 姿勢の微小な変動

## 現状分析

### 既存システム（ur_pykdl ベース）の問題点

1. **ur_pykdl の順運動学計算**
   - 内部実装の詳細が不明確
   - pinocchio と比較して検証されていない

2. **座標変換の複雑さ**
   - base frame → tool0 frame への変換が手動で実装されている
   - Coriolis 項の計算が正しいか未検証

3. **数値微分のタイミング**
   - base frame での速度を微分 → その後 tool0 frame に変換
   - 微分と座標変換の順序が結果に影響する可能性

### Pinocchio ベースの利点

1. **検証済みライブラリ**: pinocchio は広く使われ、テストされている
2. **単一関数呼び出し**: `getFrameClassicalAcceleration(LOCAL)` で直接 tool0 frame の classical acceleration を取得
3. **明確なセマンティクス**: LOCAL/WORLD/LOCAL_WORLD_ALIGNED のフレーム指定が明確

## 数学的考慮事項

### Proper Acceleration と Kinematic Acceleration

**慣性パラメータ同定**では、F/T センサが測定する力は「適正加速度 (proper acceleration)」に対応する：

```
F = m × a_proper
a_proper = a_kinematic + g_local
```

ここで：
- `a_kinematic`: 運動学的加速度（pinocchio が計算するもの）
- `g_local`: tool0 frame での重力ベクトル（`R @ [0, 0, -9.81]`）
- `a_proper`: F/T センサーが測定する力に対応する加速度

### Pinocchio での実装

```python
# 1. pinocchio で kinematic acceleration を取得 (LOCAL frame)
classical_acc = pin.getFrameClassicalAcceleration(..., LOCAL)

# 2. 重力を tool0 frame に変換して加算
gravity_base = np.array([0, 0, -9.81])
R_tool0_base = oMf.rotation.T  # base → tool0 の回転
gravity_local = R_tool0_base @ gravity_base

# 3. proper acceleration = kinematic + gravity
la_proper = classical_acc.linear + gravity_local
```

## 実装詳細

### Phase 1: tool0_kinematics.py の拡張（完了）

`compute_with_gravity()` メソッドを追加:

```python
def compute_with_gravity(self, q, v, t, gravity=np.array([0, 0, -9.81])):
    result = self.compute(q, v, t)

    oMf = self.data.oMf[self.tool0_id]
    R_local_world = oMf.rotation.T
    gravity_local = R_local_world @ gravity

    result["proper_linear_acceleration"] = result["linear_acceleration"] + gravity_local
    result["gravity_local"] = gravity_local
    result["rotation_tool0_base"] = R_local_world
    return result
```

### Phase 2: tool0_kinematics_node.py（完了）

**新規ファイル**: `scripts/tool0_kinematics_node.py`

| トピック | 型 | 内容 |
|---------|-----|------|
| `~/lv_tool0` | Vector3 | 線形速度 |
| `~/av_tool0` | Vector3 | 角速度 |
| `~/la_tool0` | Vector3 | 線形適正加速度 |
| `~/aa_tool0` | Vector3 | 角加速度 |
| `~/regressor` | Float64MultiArray | 6x10 Regressor 行列 |

### Phase 3: Launch ファイルの更新（完了）

**batch_identifier.launch** に `use_pinocchio` フラグを追加:

```xml
<launch>
    <arg name="use_pinocchio" default="true"/>

    <!-- Pinocchio ベース（デフォルト） -->
    <include file="$(find iparam_identification)/launch/tool0_kinematics.launch"
             if="$(arg use_pinocchio)"/>

    <!-- ur_pykdl ベース（レガシー） -->
    <include file="$(find iparam_identification)/launch/wrist_end_kinematics.launch"
             unless="$(arg use_pinocchio)"/>

    <node pkg="iparam_identification" type="batch_identifier.py" name="batch_identifier">
        <param name="kinematics_ns" value="/tool0_kinematics" if="$(arg use_pinocchio)"/>
        <param name="kinematics_ns" value="/wrist_end_kinematics" unless="$(arg use_pinocchio)"/>
    </node>
</launch>
```

### batch_identifier.py の変更点

1. **kinematics_ns パラメータの追加**: トピック名前空間を切り替え可能に
2. **wrench の反転を削除**: そのまま使用するように修正
3. **OLS 結果を使用**: TLS ではなく OLS の結果を publish

## ファイル変更一覧

| ファイル | 変更内容 |
|---------|---------|
| `src/utilities/tool0_kinematics.py` | `compute_with_gravity()` メソッド追加 |
| `scripts/tool0_kinematics_node.py` | **新規作成** - Pinocchioベース運動学ノード |
| `scripts/batch_identifier.py` | kinematics_ns パラメータ追加、wrench 反転削除、OLS 使用 |
| `launch/tool0_kinematics.launch` | **新規作成** - 新ノード用launch |
| `launch/batch_identifier.launch` | use_pinocchio フラグ追加 |

## 使用方法

```bash
# Pinocchio ベース（デフォルト）
roslaunch iparam_identification batch_identifier.launch

# ur_pykdl ベース（比較用）
roslaunch iparam_identification batch_identifier.launch use_pinocchio:=false
```

## 今後の課題

1. **推定精度の向上**: 現状 17% の誤差がある。動的な運動データでの検証が必要。
2. **TLS アルゴリズムの改善**: 異常値を出す原因の調査
3. **重心・慣性テンソルの検証**: 既知の形状・質量分布を持つ物体での検証
4. **オフセット補正アプローチ**: F/T センサーのゼロ点補正なしでの同定（論文の式(7)-(9)）

## 参考

- `notes/tool0_kinematics_best_practices.md` - pinocchio による速度・加速度取得の詳細
- `notes/wrench_topic_and_coordinate_frames.md` - wrench トピックと座標系の説明
