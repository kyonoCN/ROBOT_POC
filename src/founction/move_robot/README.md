# Move Robot - 虚拟机器人底盘移动控制

## 概述

`move_robot` 是一个 ROS2 功能包，提供可视化 GUI 界面来控制轮式人形机器人底盘的虚拟移动，并支持位置标记功能。

**核心功能**：
1. 速度控制：按指定速度持续移动
2. 位置控制：移动到指定位置
3. 位置标记：在空间中添加坐标轴标记，便于定位和导航

## 原理

### URDF 虚拟关节结构

```
world (固定全局坐标系)
  └── virtual_x_joint (prismatic, X方向平移)
        └── virtual_y_joint (prismatic, Y方向平移)
              └── virtual_yaw_joint (revolute, Z轴旋转)
                    └── chassis_base_link (机器人底盘)
```

### 虚拟关节定义

| 关节名称 | 类型 | 轴向 | 范围 |
|---------|------|------|------|
| virtual_x_joint | prismatic | X | ±100m |
| virtual_y_joint | prismatic | Y | ±100m |
| virtual_yaw_joint | revolute | Z | ±π rad |

### 数据流

```
move_robot_gui (本节点)
    │
    │ 发布 /joint_states (virtual_x/y/yaw_joint)
    ↓
fake_joint_driver
    │
    │ 合并所有关节状态，发布完整 /joint_states
    ↓
robot_state_publisher
    │
    │ 根据 URDF 和关节状态计算 TF
    ↓
TF Tree (world -> virtual_x_link -> ... -> chassis_base_link -> ...)
    │
    ↓
RViz 显示 + MoveIt 规划
```

### 两种控制模式

#### 模式 1：速度控制 (Velocity Mode)
- 按指定速度持续移动
- 支持 4 个平移方向：X+, X-, Y+, Y-（机器人本体坐标系）
- 支持 2 个旋转方向：Yaw+（逆时针）, Yaw-（顺时针）
- 点击方向按钮开始移动，点击 STOP 停止

#### 模式 2：位置控制 (Position Mode)
- 移动到指定的目标位置 (X, Y, Yaw)
- 可设置移动时间（duration），机器人匀速移动
- duration=0 表示瞬间移动
- 到达目标后自动停止

## 安装

```bash
# 在工作空间根目录
cd ~/ws/KITT_POC_ws

# 编译所有相关包
colcon build --packages-select robot_model move_robot wheeled_humanoid_moveit_fake

# Source 环境
source install/setup.bash
```

## 使用方法

### 启动完整系统

```bash
# 终端 1：启动 MoveIt Demo（包含机器人模型、fake_joint_driver 和 RViz）
ros2 launch wheeled_humanoid_moveit_fake demo.launch.py

# 终端 2：启动底盘移动控制 GUI
ros2 launch move_robot move_robot_gui.launch.py

# 终端 3（可选）：启动场景编辑器
ros2 launch workspace workspace_gui.launch.py
```

### 单独启动 GUI

```bash
ros2 run move_robot move_robot_gui
```

### GUI 操作说明

#### 当前位姿显示
- 显示机器人当前的 X, Y 位置（米）和 Yaw 角度（度）
- 显示当前状态：Idle / Velocity Mode Active / Moving to Target

#### 速度控制面板
1. 设置线速度 (m/s) 和角速度 (°/s)
2. 点击方向按钮开始移动：
   - `↑ X+`：向前（机器人 X 正方向）
   - `↓ X-`：向后（机器人 X 负方向）
   - `← Y+`：向左（机器人 Y 正方向）
   - `→ Y-`：向右（机器人 Y 负方向）
   - `↺ Yaw+`：逆时针旋转
   - `↻ Yaw-`：顺时针旋转
3. 点击 `STOP` 停止移动

#### 位置控制面板
1. 输入目标位置（世界坐标系）：
   - Target X (m)：目标 X 坐标
   - Target Y (m)：目标 Y 坐标
   - Target Yaw (°)：目标朝向角度
   - Duration (s)：移动时间（0 = 瞬间移动）
2. 点击 `Move to Target` 开始移动
3. 点击 `Stop` 可中途停止
4. 点击 `Set Current` 将当前位姿填入目标

#### 快捷预设
- `Origin`：回到原点 (0, 0, 0°)
- `X+1m`：移动到 X=1m 位置
- `Y+1m`：移动到 Y=1m 位置
- `90°`：旋转到 90° 朝向

## 位置标记功能

位置标记功能允许在 RViz 中可视化显示坐标轴标记，便于记录和导航到特定位置。

### 标记操作

| 按钮 | 功能 |
|------|------|
| Mark Current | 在当前机器人位置添加标记 |
| Add Marker | 手动输入 X, Y, Yaw 添加标记 |
| Delete | 删除选中的标记 |
| Rename | 重命名选中的标记 |
| Show/Hide | 切换标记的可见性 |
| Go To | 移动机器人到选中标记的位置 |

### 标记显示

- 每个标记显示为三色坐标轴（红=X, 绿=Y, 蓝=Z）
- 标记上方显示名称文字
- 列表中 ● 表示可见，○ 表示隐藏

## ROS2 接口

### 发布的话题
| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| /joint_states | sensor_msgs/JointState | 虚拟底盘关节状态 |
| /position_markers | visualization_msgs/MarkerArray | 位置标记可视化 |

### 发布的关节
| 关节名 | 类型 | 单位 |
|--------|------|------|
| virtual_x_joint | position | meters |
| virtual_y_joint | position | meters |
| virtual_yaw_joint | position | radians |

## 文件结构

```
move_robot/
├── launch/
│   └── move_robot_gui.launch.py    # Launch 文件
├── move_robot/
│   ├── __init__.py
│   └── move_robot_gui.py           # 主程序（GUI + ROS 节点）
├── resource/
│   └── move_robot
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 相关修改

### URDF 修改 (robot_model/urdf/wheel_robot.urdf)

原来的固定关节：
```xml
<joint name="fixed" type="fixed">
  <parent link="world"/>
  <child link="chassis_base_link"/>
  <origin xyz="0 0 0.265" rpy="0 0 0"/>
</joint>
```

修改为虚拟移动关节链：
```xml
<!-- Virtual X joint -->
<joint name="virtual_x_joint" type="prismatic">
  <parent link="world"/>
  <child link="virtual_x_link"/>
  <axis xyz="1 0 0"/>
  <limit lower="-100" upper="100" .../>
</joint>

<!-- Virtual Y joint -->
<joint name="virtual_y_joint" type="prismatic">
  <parent link="virtual_x_link"/>
  <child link="virtual_y_link"/>
  <axis xyz="0 1 0"/>
  <limit lower="-100" upper="100" .../>
</joint>

<!-- Virtual Yaw joint -->
<joint name="virtual_yaw_joint" type="revolute">
  <parent link="virtual_y_link"/>
  <child link="virtual_yaw_link"/>
  <origin xyz="0 0 0.265" .../>  <!-- Z高度在这里设置 -->
  <axis xyz="0 0 1"/>
  <limit lower="-3.14159" upper="3.14159" .../>
</joint>

<!-- Fixed joint to chassis -->
<joint name="chassis_base_joint" type="fixed">
  <parent link="virtual_yaw_link"/>
  <child link="chassis_base_link"/>
</joint>
```

### fake_joint_driver 修改

添加了对虚拟底盘关节的支持：
- 在 `all_joints` 列表中添加 `virtual_x_joint`, `virtual_y_joint`, `virtual_yaw_joint`
- 订阅 `/joint_states` 话题，合并来自 `move_robot` 的虚拟关节状态

## 坐标系约定

- **X 正方向**：机器人前方
- **Y 正方向**：机器人左侧
- **Z 正方向**：向上
- **Yaw 正方向**：逆时针旋转（从上往下看）

## 速度参考值

| 参数 | 建议范围 | 默认值 |
|------|----------|--------|
| 线速度 | 0.1 ~ 1.0 m/s | 0.5 m/s |
| 角速度 | 10 ~ 60 °/s | 30 °/s |

## 示例场景

### 场景 1：移动机器人到工作台前
```
1. 启动 MoveIt Demo 和 Move Robot GUI
2. 在位置控制面板输入：X=1.0, Y=0.5, Yaw=0, Duration=3.0
3. 点击 "Move to Target"
4. 机器人将在 3 秒内平滑移动到目标位置
5. 使用 MoveIt 进行手臂规划
```

### 场景 2：手动遥控探索
```
1. 在速度控制面板设置：Linear=0.3, Angular=20
2. 使用方向按钮控制机器人移动
3. 观察 RViz 中机器人位置变化
4. 点击 STOP 停止
```

## 依赖

- ROS2 Humble
- rclpy
- sensor_msgs
- python3-tkinter
- numpy

## 注意事项

1. **启动顺序**：先启动 `demo.launch.py`（包含 fake_joint_driver），再启动 `move_robot_gui`

2. **关节状态合并**：`fake_joint_driver` 会合并来自 `move_robot` 的虚拟关节状态和自身管理的机械臂关节状态

3. **MoveIt 兼容性**：虚拟底盘关节不参与 MoveIt 规划，仅用于改变机器人在世界坐标系中的位置
