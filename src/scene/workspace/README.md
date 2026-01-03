# Workspace GUI - MoveIt 场景管理工具

## 概述

Workspace GUI 是一个用于 MoveIt 规划场景管理的图形化工具，支持创建、编辑和管理碰撞物体（Box 和 Mesh），以及将物体附着（Attach）到机械臂末端进行抓取规划。

## 功能特性

- **Box 障碍物管理**：创建、编辑立方体碰撞物体
- **Mesh 障碍物管理**：加载 STL 文件作为碰撞物体
- **物体附着/分离**：将物体 Attach 到机械臂夹爪，随机械臂运动
- **配置保存/加载**：JSON 格式保存和加载场景配置
- **实时可视化**：RViz 中实时显示 Marker 和碰撞物体

## 启动方式

```bash
# 编译
colcon build --packages-select workspace

# 启动
source install/setup.bash
ros2 launch workspace workspace_gui.launch.py
```

## 界面说明

### 主界面布局

```
┌─────────────────────────────────────────────────────────────┐
│  [Save Config] [Load Config]  Mesh Sample %: [100] [Apply]  │
│                                              [Clear All]    │
├──────────────┬──────────────────────────────────────────────┤
│   Objects    │              Properties                      │
│  ┌────────┐  │  ┌─────────────────────────────────────────┐ │
│  │ box_1  │  │  │ Type: Box                               │ │
│  │ box_2  │  │  │ Name: [box_1] [Rename]                  │ │
│  │ mesh_1 │  │  │ Base Frame: [world ▼]                   │ │
│  └────────┘  │  │ Position: X[0.5] Y[0.0] Z[0.5]          │ │
│              │  │ Size: X[0.1] Y[0.1] Z[0.1]              │ │
│  [Box][Mesh] │  │ Rotation: Roll[0] Pitch[0] Yaw[0]       │ │
│  [Remove][Dup]│  │ Color: R[0.8] G[0.2] B[0.2] A[0.7]      │ │
│              │  │ ☑ Enable Collision                      │ │
│              │  │ ☑ Visible                               │ │
│              │  │ ☐ Show Pose Axes                        │ │
│              │  │                                         │ │
│              │  │ Attach to Arm:                          │ │
│              │  │ [Attach Left][Attach Right][Detach]     │ │
│              │  │ [Check MoveIt State][Force Clear All]   │ │
│              │  │ Status: Not Attached                    │ │
│              │  └─────────────────────────────────────────┘ │
└──────────────┴──────────────────────────────────────────────┘
```

### 功能按钮说明

| 按钮 | 功能 |
|------|------|
| Save Config | 保存当前场景到 JSON 文件 |
| Load Config | 从 JSON 文件加载场景 |
| Mesh Sample % | 设置 Mesh 采样率（降低可提高性能） |
| Clear All | 清除所有物体 |
| Box | 添加新的 Box 障碍物 |
| Mesh | 添加新的 Mesh 障碍物（选择 STL 文件） |
| Remove | 删除选中的物体 |
| Dup | 复制选中的物体 |

### 属性面板说明

| 属性 | 说明 |
|------|------|
| Type | 物体类型（Box/Mesh） |
| Name | 物体名称（唯一标识） |
| Base Frame | 参考坐标系（world/base_link 等） |
| Position | 位置 (X, Y, Z) 单位：米 |
| Size | 尺寸 (X, Y, Z) 单位：米（仅 Box） |
| Rotation | 旋转角度 (Roll, Pitch, Yaw) 单位：度 |
| Color | 颜色 (R, G, B, A)（仅 Box） |
| Enable Collision | 是否加入 MoveIt 碰撞检测 |
| Visible | 是否在 RViz 中显示 Marker |
| Use Bounding Box | 使用包围盒代替 Mesh（仅 Mesh） |

## Attach/Detach 功能

### 原理说明

Attach 功能用于将物体"附着"到机械臂末端，使物体随机械臂运动。这在抓取规划中非常重要：

1. **Attach（附着）**：
   - 将物体从世界坐标系（World）移动到机器人状态（Robot State）
   - 物体成为 `AttachedCollisionObject`，绑定到指定的 link（如 `Lbase_link`）
   - 设置 `touch_links` 避免物体与夹爪产生碰撞检测

2. **Detach（分离）**：
   - 将物体从机器人状态移回世界坐标系
   - 计算物体在新位置的世界坐标
   - 物体成为静态障碍物

### 坐标变换原理

```
Attach 时：
  offset = inverse(T_link_world) * T_obj_world
  保存 offset 用于 detach

Detach 时：
  T_obj_world_new = T_link_world_new * offset
  即：new_pos = link_pos + link_rot * offset_pos
```

### 使用流程

1. **创建物体**：点击 "Box" 或 "Mesh" 创建障碍物
2. **调整位置**：设置物体位置到夹爪附近
3. **Attach**：点击 "Attach Left" 或 "Attach Right"
4. **规划运动**：使用 MoveIt 规划机械臂运动
5. **Detach**：到达目标位置后点击 "Detach"

### 配置说明

Attach 配置在代码中定义：

```python
ATTACH_CONFIG = {
    'left': {
        'link': 'Lbase_link',  # 左臂夹爪基座
        'touch_links': [       # 允许接触的 links
            'Lbase_link', 'Lleftout_Link', 'Lrightout_Link',
            'Lleftinn_Link', 'Lrightinn_Link', ...
        ]
    },
    'right': {
        'link': 'Rbase_link',  # 右臂夹爪基座
        'touch_links': [...]
    }
}
```

## 配置文件格式

### JSON 配置示例

```json
{
  "obstacles": [
    {
      "type": "box",
      "name": "table",
      "position": [0.6, 0.0, 0.35],
      "size": [0.6, 0.8, 0.02],
      "rotation_euler": [0.0, 0.0, 0.0],
      "color": [0.6, 0.4, 0.2, 0.8],
      "base_frame": "world",
      "collision_enabled": true,
      "visible": true,
      "pose_axis_visible": false,
      "attached_to": null
    }
  ],
  "meshes": [
    {
      "type": "mesh",
      "name": "object_1",
      "mesh_path": "base_link.STL",
      "position": [0.5, 0.0, 0.5],
      "rotation_euler": [0.0, 0.0, 0.0],
      "scale": [1.0, 1.0, 1.0],
      "base_frame": "world",
      "collision_enabled": true,
      "use_bbox": false,
      "attached_to": null
    }
  ]
}
```

### 配置字段说明

| 字段 | 类型 | 说明 |
|------|------|------|
| type | string | "box" 或 "mesh" |
| name | string | 物体唯一名称 |
| position | [x,y,z] | 位置（米） |
| size | [x,y,z] | 尺寸（米，仅 box） |
| rotation_euler | [r,p,y] | 欧拉角（度） |
| color | [r,g,b,a] | 颜色（0-1，仅 box） |
| base_frame | string | 参考坐标系 |
| collision_enabled | bool | 是否启用碰撞 |
| visible | bool | 是否可见（仅 box） |
| mesh_path | string | STL 文件路径（仅 mesh） |
| scale | [x,y,z] | 缩放比例（仅 mesh） |
| use_bbox | bool | 使用包围盒（仅 mesh） |
| attached_to | string/null | 附着的机械臂（left/right/null） |

## ROS2 接口

### 发布的 Topics

| Topic | 类型 | 说明 |
|-------|------|------|
| /workspace_markers | MarkerArray | 物体可视化 Marker |
| /workspace_pose_axes | MarkerArray | 坐标轴可视化 |

### 使用的 Services

| Service | 类型 | 说明 |
|---------|------|------|
| /apply_planning_scene | ApplyPlanningScene | 更新 MoveIt 规划场景 |
| /get_planning_scene | GetPlanningScene | 查询 MoveIt 规划场景状态 |

### TF 依赖

- 使用 TF2 进行坐标变换
- 需要机器人 URDF 发布的 TF 树

## 目录结构

```
workspace/
├── config/                    # 配置文件目录
│   ├── default_workspace.json
│   └── ws_1213_2.json
├── launch/
│   └── workspace_gui.launch.py
├── mesh/                      # STL 文件目录
│   └── base_link.STL
├── workspace/
│   ├── __init__.py
│   ├── workspace_gui.py       # 主程序
│   └── test_attach_detach.py  # 测试脚本
├── package.xml
├── setup.py
└── README.md
```

## 常见问题

### Q: Detach 后物体位置不正确？

A: 确保在 Attach 时 TF 树正常工作。程序会在 Attach 时保存物体相对于 link 的偏移量，Detach 时使用该偏移量计算新位置。

### Q: MoveIt 显示物体仍然 Attached？

A: 点击 "Check MoveIt State" 查看实际状态，如有问题可点击 "Force Clear All" 强制清除所有附着物体。

### Q: Mesh 加载很慢？

A: 降低 "Mesh Sample %" 值（如 50%）可以减少三角面数量，提高性能。

## 依赖

- ROS2 Humble
- MoveIt2
- Python 3.10+
- tkinter
- numpy
- scipy

## 作者

KITT POC 项目组
