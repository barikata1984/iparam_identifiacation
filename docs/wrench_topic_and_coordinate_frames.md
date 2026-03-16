# Wrench トピックと座標系

本ドキュメントでは、UR5e ロボットにおける wrench（力/トルク）データの取得方法と、関連する座標系について説明する。

## 1. 概要

UR5e には内蔵の力/トルクセンサーがあり、そのデータは ROS トピックとして公開される。wrench データを正しく解釈するには、参照座標系を理解することが重要である。

## 2. Wrench トピックのパブリッシュ元

### 2.1 データフロー

```
UR Robot (内蔵FTセンサー)
    │
    ▼ (ur_robot_driver / ros_control)
┌───────────────────────────────┐
│  /wrench                      │  ← 生データ (500Hz)
│  type: WrenchStamped          │
└───────────────────────────────┘
    │
    ▼ (ft_filter.py)
┌───────────────────────────────┐
│  /wrench/filtered             │  ← フィルタ済み
└───────────────────────────────┘
```

### 2.2 パブリッシュ元の詳細

#### `/wrench` トピック（生データ）

- **発行元**: `ur_robot_driver` の `force_torque_sensor_controller`
- **実装ファイル**: `underlay_ws/src/third_party/ur_robot_driver/ur_robot_driver/src/hardware_interface.cpp`
- **メカニズム**: `ForceTorqueSensorHandle` として ros_control に登録
- **設定ファイル**: `catkin_ws/src/osx_core/osx_ur5e/config/ur5e_controllers.yaml`

```yaml
force_torque_sensor_controller:
   type: force_torque_sensor_controller/ForceTorqueSensorController
   publish_rate: 500  # Hz
```

#### `/wrench/filtered` トピック（フィルタ済み）

- **発行元**: `ft_filter.py`（ur_control パッケージ）
- **実装ファイル**: `underlay_ws/src/ur_python_utilities/ur_control/scripts/ft_filter.py`
- **処理内容**: Butterworth ローパスフィルタを適用
- **提供サービス**:
  - `/wrench/filtered/zero_ftsensor`: ゼロ点補正
  - `/wrench/filtered/enable_filtering`: フィルタ有効/無効切替

## 3. 座標系

### 3.1 UR5e の主要フレーム階層

```
base_link (= base)
    └── shoulder_link
            └── upper_arm_link
                    └── forearm_link
                            └── wrist_1_link
                                    └── wrist_2_link
                                            └── wrist_3_link
                                                    └── flange
                                                            └── tool0
```

### 3.2 tool0 フレームの定義

URDF（`underlay_ws/src/third_party/universal_robot/ur_description/urdf/inc/ur_macro.xacro`）での定義:

```xml
<!-- ROS-Industrial 'flange' frame: attachment point for EEF models -->
<joint name="${prefix}wrist_3-flange" type="fixed">
  <parent link="${prefix}wrist_3_link" />
  <child link="${prefix}flange" />
  <origin xyz="0 0 0" rpy="0 ${-pi/2.0} ${-pi/2.0}" />
</joint>

<!-- ROS-Industrial 'tool0' frame: all-zeros tool frame -->
<joint name="${prefix}flange-tool0" type="fixed">
  <!-- default toolframe: X+ left, Y+ up, Z+ front -->
  <origin xyz="0 0 0" rpy="${pi/2.0} 0 ${pi/2.0}"/>
  <parent link="${prefix}flange"/>
  <child link="${prefix}tool0"/>
</joint>
```

### 3.3 tool0 の軸方向（ROS-Industrial 標準）

```
        Z+ (ツール前方/ロボットから離れる方向)
        ↑
        │
        │    Y+ (上)
        │   ╱
        │  ╱
        │ ╱
        ├───────→ X+ (左)
       ●
    (tool0 原点 = フランジ面中心)
```

| 軸 | 方向 |
|----|------|
| X+ | 左方向 |
| Y+ | 上方向 |
| Z+ | ツール方向（フランジから外向き） |
| 原点 | フランジ面中心（wrist_3_link と同一位置） |

### 3.4 tool0 vs tool0_controller

| フレーム | 説明 | データソース |
|----------|------|--------------|
| `tool0` | URDF で定義された固定フレーム | 順運動学計算（関節角度から算出） |
| `tool0_controller` | UR コントローラーが報告する TCP | RTDE 経由（UR コントローラーから直接） |

**TCP オフセットが設定されていない場合（デフォルト）、両者は一致する。**

確認コマンド:
```bash
# 両フレームを比較
rosrun tf tf_echo base tool0
rosrun tf tf_echo base tool0_controller
```

実測例（ロボット静止状態）:
```
# tool0 (URDF)
Translation: [0.114, -0.139, 0.631]
RPY (degree): [-178.050, -1.796, 110.385]

# tool0_controller (UR)
Translation: [0.115, -0.139, 0.632]
RPY (degree): [-177.989, -1.784, 110.488]
```

差分は約 1mm、0.1° 程度であり、これはキャリブレーション誤差・計算タイミング差による。

## 4. Wrench データの座標系

### 4.1 UR ドライバーでの座標変換

`hardware_interface.cpp` での処理（e-Series の場合）:

```cpp
// rotate f/t sensor output back to the flange frame
ft = base_to_flange.M.Inverse() * ft;

// Transform the wrench to the tcp frame
ft = flange_to_tcp * ft;
```

すなわち、`/wrench` トピックで公開される wrench データは **TCP（Tool Center Point）フレーム** で表現されている。

### 4.2 TCP フレームと tool0 フレームの関係

| 条件 | TCP と tool0 の関係 |
|------|---------------------|
| TCP オフセット = ゼロ（デフォルト） | TCP = tool0（一致） |
| TCP オフセットを設定済み | TCP ≠ tool0（ずれる） |

### 4.3 結論

**TCP オフセットがゼロ（デフォルト）の場合**:

- `/wrench` および `/wrench/filtered` トピックは **tool0 フレーム基準**
- 原点: フランジ中心
- Z+: ツール方向（フランジから外向き）

## 5. TCP 設定

### 5.1 TCP オフセットの設定場所

TCP は **UR コントローラー側**で設定し、ROS ドライバーは RTDE 経由でその値を読み取るのみである。

設定方法:
1. **Teach Pendant**: `Installation > General > TCP`
2. **URScript**: `set_tcp(p[x, y, z, rx, ry, rz])`
3. **Installation ファイル**: `.installation` ファイルに保存

### 5.2 ROS ドライバーでの読み取り

`hardware_interface.cpp`:
```cpp
readData(data_pkg, "tcp_offset", tcp_offset_);
```

RTDE プロトコル経由で `tcp_offset` を取得し、wrench の座標変換に使用している。

### 5.3 注意事項

URDF 上の `ee_link`（通常 `tool0`）と UR コントローラーの TCP 設定が一致している前提で制御が行われる。Teach Pendant で TCP オフセットを変更した場合、不整合が生じる可能性がある。

## 6. Python API での Wrench 取得

### 6.1 クラス階層

```
RobotInterface (osx_bilateral/_src/teleop/robot.py)
    └── self._arm: CompliantController
                        └── 継承: Arm (ur_control/src/ur_control/arm.py)
                                   └── get_wrench() メソッド
```

### 6.2 Arm.get_wrench() メソッド

`underlay_ws/src/ur_python_utilities/ur_control/src/ur_control/arm.py`:

```python
def get_wrench(self,
               base_frame_control=False,
               hand_frame_control=False) -> np.ndarray:
    """
    Returns the wrench (force/torque) in task-space.
    By default, return the wrench as read from the sensor topic.

    Parameters
    ----------
    base_frame_control : bool, optional
        If True, returns the wrench with respect to the robot base frame
    hand_frame_control : bool, optional
        If True, returns the wrench with respect to the end-effector frame
        If both base_frame_control and hand_frame_control are set to True,
        the former is considered.

    Returns
    -------
    res : np.ndarray
        Returns the wrench as [fx, fy, fz, tx, ty, tz]
    """
```

### 6.3 座標変換の内部処理

`arm.py` での座標変換:

```python
if base_frame_control:
    # Transform force/torque from sensor to robot base frame
    transform = self.end_effector(tip_link=self.joint_names_prefix + "wrist_3_link")
    ee_wrench_force = spalg.convert_wrench(wrench_force, transform)
    return ee_wrench_force
else:
    # Transform force/torque from sensor to end effector frame
    transform = self.end_effector(tip_link=self.ee_link)
    ee_wrench_force = spalg.convert_wrench(wrench_force, transform)
    return ee_wrench_force
```

座標変換には `spalg.convert_wrench()` が使用される（`ur_control/src/ur_control/spalg.py`）。

### 6.4 使用例

```python
from ur_control.arm import Arm

arm = Arm(ft_topic="wrench", ee_link="tool0")

# センサーフレーム（生データ、TCP フレーム）
wrench_raw = arm.get_wrench()

# ベースフレーム基準に変換
wrench_base = arm.get_wrench(base_frame_control=True)

# エンドエフェクタフレーム（tool0）基準に変換
wrench_ee = arm.get_wrench(hand_frame_control=True)
```

### 6.5 RobotInterface 経由での取得

`osx_bilateral/_src/teleop/robot.py`:

```python
def get_wrench(self) -> np.ndarray:
    """Get force/torque sensor reading.

    Returns:
        Wrench as [fx, fy, fz, tx, ty, tz]
    """
    if self._arm is None:
        return np.zeros(6)
    return self._arm.get_wrench(base_frame_control=True)  # ベースフレーム固定
```

**注意**: `RobotInterface.get_wrench()` は常に `base_frame_control=True` で呼び出される。

## 7. トピック購読での座標変換

`arm.get_wrench()` を使わずにトピックを直接購読する場合、TF2 を使用して任意のフレームに変換可能。

```python
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import WrenchStamped

# TF バッファの準備
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

def wrench_callback(msg: WrenchStamped):
    # 任意のフレームに変換（例: base_link）
    try:
        transformed = tf_buffer.transform(msg, "base_link")
        # transformed.wrench を使用
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
        pass

rospy.Subscriber("/wrench", WrenchStamped, wrench_callback)
```

## 8. FT センサーのゼロ点補正

### 8.1 利用可能なサービス

| サービス | 説明 |
|----------|------|
| `/ur_hardware_interface/zero_ftsensor` | UR ドライバー側のゼロ点補正 |
| `/wrench/filtered/zero_ftsensor` | フィルタ側のゼロ点補正 |

### 8.2 Python API

```python
arm.zero_ft_sensor()  # 両方のゼロ点補正を実行
```

`arm.py` での実装:
```python
def zero_ft_sensor(self, sleep_time=0.05):
    """Reset force-torque sensor readings to zeros."""
    if not rospy.has_param("use_gazebo_sim"):
        # First try to zero FT from ur_driver
        self._zero_ft()
        rospy.sleep(sleep_time)
    # Then update filtered one
    self._zero_ft_filtered()
    rospy.sleep(sleep_time)
```

## 9. 関連ファイル一覧

| ファイル | 説明 |
|----------|------|
| `underlay_ws/src/third_party/ur_robot_driver/ur_robot_driver/src/hardware_interface.cpp` | FT データの取得と座標変換 |
| `underlay_ws/src/ur_python_utilities/ur_control/src/ur_control/arm.py` | Arm クラス、get_wrench() メソッド |
| `underlay_ws/src/ur_python_utilities/ur_control/src/ur_control/fzi_cartesian_compliance_controller.py` | CompliantController クラス |
| `underlay_ws/src/ur_python_utilities/ur_control/scripts/ft_filter.py` | フィルタリングノード |
| `underlay_ws/src/ur_python_utilities/ur_control/src/ur_control/spalg.py` | 座標変換ユーティリティ（convert_wrench） |
| `underlay_ws/src/ur_python_utilities/ur_control/src/ur_control/constants.py` | 定数定義（EE_LINK, FT_SUBSCRIBER 等） |
| `catkin_ws/src/osx_bilateral/_src/teleop/robot.py` | RobotInterface クラス |
| `underlay_ws/src/third_party/universal_robot/ur_description/urdf/inc/ur_macro.xacro` | URDF フレーム定義 |
| `catkin_ws/src/osx_core/osx_ur5e/config/ur5e_controllers.yaml` | コントローラー設定 |

## 10. まとめ

1. `/wrench` および `/wrench/filtered` トピックは **TCP フレーム**で wrench を報告する
2. TCP オフセットがゼロ（デフォルト）の場合、**TCP = tool0**（フランジ中心、Z+ 外向き）
3. TCP 設定は UR コントローラー側（Teach Pendant 等）で行い、ROS ドライバーは RTDE 経由で読み取る
4. Python API では `arm.get_wrench()` で以下の座標系オプションを指定可能:
   - 引数なし: センサーフレーム（TCP フレーム）
   - `base_frame_control=True`: ロボットベースフレーム
   - `hand_frame_control=True`: エンドエフェクタフレーム（ee_link）
5. `tool0` と `tool0_controller` の TF 比較で TCP オフセットの有無を確認可能
6. 現在の構成では TCP オフセット = ゼロであり、wrench はフランジ中心・Z+ 外向きの tool0 座標系で表現されている
