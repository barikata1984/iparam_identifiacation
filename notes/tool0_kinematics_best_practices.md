# tool0 座標系における速度・加速度取得のベストプラクティス

本ドキュメントは、UR5e ロボットの tool0 フレームにおける線速度・角速度・線加速度・角加速度を取得するための推奨手法をまとめたものである。慣性パラメータ同定を最終目標とし、鶏と卵の問題を回避するアプローチを採用する。

## 1. 前提条件

### 1.1 利用可能なライブラリ

- **pinocchio** v3.6.0: ロボット運動学・動力学ライブラリ
- **numpy**: 数値計算
- **ROS**: `/joint_states` トピックからの関節状態取得

### 1.2 `/joint_states` トピックの仕様

UR ドライバがパブリッシュする `/joint_states` トピックの仕様:

| フィールド | 内容 | 利用 |
|-----------|------|------|
| `position` | 関節位置 [rad] | ✓ 使用 |
| `velocity` | 関節速度 [rad/s] | ✓ 使用（直接取得） |
| `effort` | 関節トルク [Nm] | （慣性同定の入力として別途使用） |

**注意事項**:
- パブリッシュ周波数: 約 **500 Hz**（約 2ms 間隔）
- 関節名の順序は UR ドライバの実装により異なる場合がある（例: `elbow_joint` が先頭）
- コード内で関節名を明示的に指定して正しい順序で取得すること

### 1.3 tool0 フレームの定義

tool0 フレームは UR5e のフランジ中心に位置し、Z軸がツール方向（外向き）を指す。pinocchio モデルにおける tool0 のフレーム ID は **21** である。

```python
import pinocchio as pin

model = pin.buildModelFromUrdf(urdf_path)
tool0_id = model.getFrameId("tool0")  # = 21
```

## 2. 空間速度 vs 古典的速度・加速度

### 2.1 定義の違い

Lynch & Park "Modern Robotics" の定義に基づく:

| 量 | 空間的 (Spatial) | 古典的 (Classical) |
|----|------------------|-------------------|
| 線速度 | ツイストの線速度成分 $v$ | 位置の1階時間微分 $\dot{p}$ |
| 線加速度 | ツイスト微分の線速度成分 $\dot{v}$ | 位置の2階時間微分 $\ddot{p}$ |
| 角速度 | $\omega$ | $\omega$ (同一) |
| 角加速度 | $\dot{\omega}$ | $\dot{\omega}$ (同一) |

### 2.2 pinocchio が返す量

pinocchio の各関数が返す量は以下の通り:

| 関数 | 返り値 | 内容 |
|------|--------|------|
| `getFrameVelocity()` | **空間速度（ツイスト）** | V = (ω, v) |
| `getFrameAcceleration()` | **空間加速度** | V̇ = (ω̇, v̇) |
| `getFrameClassicalAcceleration()` | **古典的加速度** | (ω̇, p̈) |

**重要**: `getFrameVelocity()` と `getFrameAcceleration()` は**空間的**な量を返す。古典的な線加速度（位置の2階微分）が必要な場合は `getFrameClassicalAcceleration()` を使用する。

### 2.3 重要な関係式

**LOCAL フレーム（tool0 座標系）において**:
- 空間線速度 = 古典的線速度（一致する）
- 空間線加速度 ≠ 古典的線加速度

古典的線加速度と空間線加速度の関係:
$$\ddot{p} = \dot{v} + \omega \times v$$

### 2.4 慣性パラメータ同定に必要な量

Newton-Euler 方程式:
$$F = m \ddot{p}_{cm} + m g$$
$$\tau = I \dot{\omega} + \omega \times (I \omega)$$

ここで必要なのは**古典的**線加速度 $\ddot{p}$ と角加速度 $\dot{\omega}$ である。

## 3. 推奨実装

### 3.1 関節速度・加速度の取得方針

**方針**:
- **関節速度**: `/joint_states` の `velocity` フィールドから直接取得
- **関節加速度**: 関節速度を**数値微分**して取得

**順動力学を使用しない理由**:
- `/joint_states` の effort（関節トルク）は操作対象物の慣性の影響を含む
- 対象物の慣性を知らずに順動力学を解くことは不可能（鶏と卵の問題）
- 数値微分は純粋な運動学的アプローチであり、動力学モデルに依存しない

**`/joint_states` の `velocity` を直接利用する利点**:
- 数値微分が1回（v → a）で済むため、ノイズの蓄積が軽減される
- 位置→速度の微分で生じる位相遅れが解消される
- UR ドライバが内部で計算した高精度な速度値を利用できる

```python
import numpy as np

class NumericalDifferentiator:
    """ローパスフィルタ付き数値微分器"""

    def __init__(self, cutoff_freq: float = 10.0):
        self.cutoff_freq = cutoff_freq
        self.prev_value = None
        self.prev_time = None
        self.prev_derivative = None

    def update(self, value: np.ndarray, time: float) -> np.ndarray:
        if self.prev_value is None:
            self.prev_value = value
            self.prev_time = time
            self.prev_derivative = np.zeros_like(value)
            return self.prev_derivative

        dt = time - self.prev_time
        if dt <= 0:
            return self.prev_derivative

        # 生の微分値
        raw_derivative = (value - self.prev_value) / dt

        # 1次ローパスフィルタ
        alpha = dt * self.cutoff_freq / (1 + dt * self.cutoff_freq)
        filtered_derivative = alpha * raw_derivative + (1 - alpha) * self.prev_derivative

        self.prev_value = value
        self.prev_time = time
        self.prev_derivative = filtered_derivative

        return filtered_derivative

    def reset(self):
        self.prev_value = None
        self.prev_time = None
        self.prev_derivative = None
```

### 3.2 pinocchio による tool0 速度・加速度の取得

```python
import pinocchio as pin
import numpy as np

class Tool0KinematicsCalculator:
    """tool0 フレームの速度・加速度を計算するクラス"""

    def __init__(self, urdf_path: str, acc_cutoff_freq: float = 10.0):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.tool0_id = self.model.getFrameId("tool0")

        # 関節加速度用の数値微分器（速度→加速度の1段階のみ）
        self.acc_differentiator = NumericalDifferentiator(cutoff_freq=acc_cutoff_freq)

    def compute(self, q: np.ndarray, v: np.ndarray, t: float) -> dict:
        """
        関節位置・速度と時刻から tool0 の速度・加速度を計算する。

        Parameters
        ----------
        q : np.ndarray
            関節位置 [rad], shape (6,)
        v : np.ndarray
            関節速度 [rad/s], shape (6,)（/joint_states から直接取得）
        t : float
            タイムスタンプ [s]

        Returns
        -------
        dict
            tool0 座標系で記述された速度・加速度
        """
        # 数値微分で関節加速度を取得（速度→加速度の1段階のみ）
        a = self.acc_differentiator.update(v, t)

        # 順運動学の計算（位置・速度・加速度を伝播）
        pin.forwardKinematics(self.model, self.data, q, v, a)
        pin.updateFramePlacements(self.model, self.data)

        # tool0 座標系 (LOCAL) での空間速度（ツイスト）
        twist = pin.getFrameVelocity(
            self.model, self.data, self.tool0_id,
            pin.ReferenceFrame.LOCAL
        )

        # tool0 座標系 (LOCAL) での古典的加速度
        classical_acc = pin.getFrameClassicalAcceleration(
            self.model, self.data, self.tool0_id,
            pin.ReferenceFrame.LOCAL
        )

        return {
            # 速度（LOCAL フレームでは空間速度 = 古典的速度）
            "angular_velocity": twist.angular.copy(),      # ω [rad/s]
            "linear_velocity": twist.linear.copy(),        # ṗ [m/s]

            # 古典的加速度（位置の2階微分）
            "angular_acceleration": classical_acc.angular.copy(),  # ω̇ [rad/s²]
            "linear_acceleration": classical_acc.linear.copy(),    # p̈ [m/s²]

            # 生データ（デバッグ用）
            "joint_position": q.copy(),
            "joint_velocity": v.copy(),
            "joint_acceleration": a.copy(),
        }

    def reset(self):
        """数値微分器の状態をリセット"""
        self.acc_differentiator.reset()
```

### 3.3 ROS ノードでの使用例

```python
#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState

# 関節名の順序（pinocchio モデルの順序に合わせる）
# 注意: /joint_states の関節名の順序は UR ドライバにより異なる場合がある
#       （例: elbow_joint が先頭に来る場合あり）
JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

class Tool0KinematicsNode:
    def __init__(self):
        rospy.init_node("tool0_kinematics_node")

        urdf_path = rospy.get_param("~urdf_path")
        acc_cutoff_freq = rospy.get_param("~acc_cutoff_freq", 10.0)
        self.calculator = Tool0KinematicsCalculator(urdf_path, acc_cutoff_freq)

        self.sub = rospy.Subscriber(
            "/joint_states", JointState, self.joint_state_callback
        )

    def joint_state_callback(self, msg: JointState):
        # 関節状態を正しい順序で取得
        q = np.zeros(6)
        v = np.zeros(6)
        for i, name in enumerate(JOINT_ORDER):
            if name in msg.name:
                idx = msg.name.index(name)
                q[i] = msg.position[idx]
                v[i] = msg.velocity[idx]

        t = msg.header.stamp.to_sec()

        # tool0 の速度・加速度を計算
        result = self.calculator.compute(q, v, t)

        # 結果の利用（例: パブリッシュ、ログ等）
        rospy.loginfo_throttle(1.0, f"ω: {result['angular_velocity']}")
        rospy.loginfo_throttle(1.0, f"ṗ: {result['linear_velocity']}")
        rospy.loginfo_throttle(1.0, f"ω̇: {result['angular_acceleration']}")
        rospy.loginfo_throttle(1.0, f"p̈: {result['linear_acceleration']}")

if __name__ == "__main__":
    node = Tool0KinematicsNode()
    rospy.spin()
```

## 4. 参照座標系の選択

pinocchio の `ReferenceFrame` オプション:

| オプション | 説明 | 用途 |
|-----------|------|------|
| `LOCAL` | tool0 座標系で記述 | **FTセンサデータとの統合に推奨** |
| `LOCAL_WORLD_ALIGNED` | tool0 位置、ワールド姿勢で記述 | 重力方向との比較に便利 |
| `WORLD` | ワールド座標系で記述 | グローバルな運動解析 |

**推奨**: FTセンサは通常 tool0 フレームに取り付けられているため、`LOCAL` フレームを使用することで座標変換なしにデータを統合できる。

## 5. 注意事項

### 5.1 数値微分のノイズ

数値微分は高周波ノイズに敏感である。対策:
- ローパスフィルタの適用（上記実装に含まれる）
- カットオフ周波数の調整（デフォルト 10 Hz）
- より高度なフィルタ（Savitzky-Golay 等）の検討

### 5.2 初期化時の不安定性

数値微分器は初期の数サンプルで不安定な値を出力する。対策:
- 計測開始後、一定時間はデータを破棄する
- `reset()` 後の最初の数サンプルを無視する

### 5.3 URDF パス

このプロジェクトで利用可能な UR5e URDF:
```
/root/osx-ur/underlay_ws/src/ur_python_utilities/ur_pykdl/urdf/ur5e.urdf
```

## 6. 検証方法

実装の正しさを検証するための手法:

1. **静止状態テスト**: ロボット静止時に速度・加速度がゼロ近傍であることを確認
2. **一定速度テスト**: 一定速度運動時に加速度がゼロ近傍であることを確認
3. **既知軌道テスト**: 解析的に速度・加速度が既知の軌道で比較検証
4. **空間加速度との整合性**: `getFrameAcceleration` の結果と `ω × v` の補正項を加えた値が `getFrameClassicalAcceleration` と一致することを確認

```python
# 検証コード例
spatial_acc = pin.getFrameAcceleration(model, data, tool0_id, pin.LOCAL)
classical_acc = pin.getFrameClassicalAcceleration(model, data, tool0_id, pin.LOCAL)
twist = pin.getFrameVelocity(model, data, tool0_id, pin.LOCAL)

# この等式が成り立つことを確認
# classical_acc.linear ≈ spatial_acc.linear + np.cross(twist.angular, twist.linear)
computed_classical_linear = spatial_acc.linear + np.cross(twist.angular, twist.linear)
np.testing.assert_allclose(classical_acc.linear, computed_classical_linear, atol=1e-10)
```

## 7. 参考文献

- Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press. (Chapter 8)
- Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control*. Pearson. (Chapter 6)
- pinocchio documentation: https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/
