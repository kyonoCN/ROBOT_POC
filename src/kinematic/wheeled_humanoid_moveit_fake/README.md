# wheeled_humanoid_moveit_fake

虚拟机器人 MoveIt2 配置包，用于离线仿真和开发。

## 架构说明（方法二：模拟 ros2_control 流）

本包采用"模拟 ros2_control"的架构，**与真实机器人的软件栈完全同构**，只是把硬件层换成虚拟实现。

### 数据流架构

```
┌─────────────────────────────────────────────────────────┐
│                    RViz2 (可视化)                        │
│  - 显示机器人模型（通过 TF）                              │
│  - MoveIt 插件（规划、执行界面）                          │
└─────────────────────────────────────────────────────────┘
                         ↑
                         │ TF + /robot_description
                         │
┌─────────────────────────────────────────────────────────┐
│              robot_state_publisher                       │
│  - 订阅 /joint_states                                    │
│  - 发布 TF 变换                                          │
└─────────────────────────────────────────────────────────┘
                         ↑
                         │ /joint_states (100Hz)
                         │
┌─────────────────────────────────────────────────────────┐
│         fake_joint_driver (模拟 ros2_control)            │
│  - 发布 /joint_states（类似 joint_state_broadcaster）    │
│  - 提供 FollowJointTrajectory action servers            │
│    （类似 JointTrajectoryController）                    │
│  - 平滑插值执行轨迹                                       │
└─────────────────────────────────────────────────────────┘
                         ↑
                         │ FollowJointTrajectory action
                         │
┌─────────────────────────────────────────────────────────┐
│                 move_group (MoveIt)                      │
│  - 接收规划请求（OMPL/Pilz）                              │
│  - 通过 MoveItSimpleControllerManager 发送轨迹           │
│  - 订阅 /joint_states 验证执行状态                        │
└─────────────────────────────────────────────────────────┘
```

### 与真实机器人对比

| 组件 | 真实机器人 | 虚拟机器人（本包） |
|------|-----------|-------------------|
| 硬件接口 | 真实驱动 + CAN/EtherCAT | fake_joint_driver（纯软件模拟） |
| joint_state_broadcaster | ros2_control | fake_joint_driver |
| JointTrajectoryController | ros2_control | fake_joint_driver |
| move_group | MoveIt | MoveIt（相同配置） |
| 控制器名称 | left_arm_controller 等 | **完全相同** |

## 特点

- **控制器名称与真实机器人完全一致**：确保在虚拟机器人上开发的功能可以直接部署到真实机器人
- **完整模拟 ros2_control 数据流**：有 `/joint_states`、FollowJointTrajectory action
- **无需真实硬件连接**
- **平滑轨迹执行**：使用 100Hz 线性插值
- 可在 RViz 中进行 Plan & Execute 操作

## 控制器名称映射

| 控制器名称 | 功能 | 关节 |
|-----------|------|------|
| `left_arm_controller` | 左臂轨迹控制 | AR5_5_07L_joint_1-7 |
| `right_arm_controller` | 右臂轨迹控制 | AR5_5_07R_joint_1-7 |
| `torso_controller` | 躯干控制 | Trunk_Joint1-4 |
| `head_controller` | 头部控制 | Head_Joint1-2 |
| `left_gripper_controller` | 左夹爪控制 | Lfinger_joint |
| `right_gripper_controller` | 右夹爪控制 | Rfinger_joint |

## 使用方法

### 编译

```bash
cd ~/ws/KITT_ws
colcon build --packages-select wheeled_humanoid_moveit_fake
source install/setup.bash
```

### 启动虚拟机器人

```bash
ros2 launch wheeled_humanoid_moveit_fake demo.launch.py
```

### 可选参数

```bash
# 启动时打开关节滑块GUI
ros2 launch wheeled_humanoid_moveit_fake demo.launch.py use_gui:=true

# 不启动RViz（用于headless模式）
ros2 launch wheeled_humanoid_moveit_fake demo.launch.py use_rviz:=false
```

## 与真实机器人的关系

```
wheeled_humanoid_moveit_fake (本包)     wheeled_humanoid_moveit (真实机器人)
├── config/                              ├── config/
│   └── moveit_controllers.yaml          │   ├── moveit_controllers.yaml  ← 控制器名称一致！
│       (控制器名称一致)                  │   ├── robot_model.srdf         ← 共享
├── scripts/                             │   ├── kinematics.yaml          ← 共享
│   └── fake_joint_driver.py             │   └── joint_limits.yaml        ← 共享
└── launch/                              └── launch/
    └── demo.launch.py                       └── demo.launch.py
```

## 开发工作流

1. 在虚拟机器人上开发和测试功能
2. 功能验证通过后，直接切换到真实机器人运行
3. 无需修改任何控制器相关代码

```bash
# 开发阶段 - 使用虚拟机器人
ros2 launch wheeled_humanoid_moveit_fake demo.launch.py

# 部署阶段 - 使用真实机器人 (修改domain连接真实机器人)
ros2 launch wheeled_humanoid_moveit demo.launch.py
```

## 规划组 (Planning Groups)

| 规划组 | 描述 |
|--------|------|
| `left_arm` | 左臂 7 自由度 |
| `right_arm` | 右臂 7 自由度 |
| `dual_arm` | 双臂协同 |
| `torso` | 躯干 4 自由度 |
| `torso_with_left_arm` | 躯干 + 左臂 |
| `torso_with_right_arm` | 躯干 + 右臂 |
| `torso_with_dual_arm` | 躯干 + 双臂 |
| `head` | 头部 2 自由度 |
| `left_gripper` | 左夹爪 |
| `right_gripper` | 右夹爪 |

## 预设姿态 (Named States)

- `left_arm_zero`, `right_arm_zero`, `dual_arm_zero` - 零位姿态
- `left_arm_home`, `right_arm_home`, `dual_arm_home` - 初始姿态
- `left_arm_hang`, `right_arm_hang`, `dual_arm_hang` - 悬挂姿态
- `torso_home` - 躯干初始姿态
- `head_home` - 头部初始姿态
- `left_gripper_open/close`, `right_gripper_open/close` - 夹爪开合
