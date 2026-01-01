# enemy_tracking

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## 1. 项目介绍

RoboMaster 哨兵机器人敌人追踪导航功能包，基于视觉追踪结果和 Nav2 实现自主追击敌人。

核心功能：
- 订阅视觉追踪目标（`rm_interfaces/msg/Target`）
- 在敌人周围生成候选攻击点（圆形分布）
- 使用 costmap 过滤不可行的点（避开障碍物）
- 选择离机器人最近的可行攻击点
- 自动计算朝向敌人的攻击姿态
- 调用 Nav2 导航到攻击位置

算法参考：
- [pb2025_sentry_behavior](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior) 的 `CalculateAttackPose` 行为树插件

## 2. 项目依赖

- [rm_interfaces](https://github.com/your-repo/rm_interfaces): 视觉自定义 ROS 消息接口
- [Nav2](https://github.com/ros-planning/navigation2): ROS2 导航框架

## 3. Quick Start

### 3.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Nav2

### 3.2 Build

```bash
cd ~/ros_ws/src
git clone https://github.com/your-repo/enemy_tracking.git
```

```bash
cd ~/ros_ws
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --packages-select enemy_tracking
```

### 3.3 Running

```bash
ros2 launch enemy_tracking enemy_tracking.launch.py
```

## 4. 节点说明

### 4.1 订阅话题

| 话题 | 类型 | 描述 |
|-|-|-|
| `armor_solver/target` | `rm_interfaces/msg/Target` | 视觉追踪目标 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 全局代价地图 |

### 4.2 发布话题

| 话题 | 类型 | 描述 |
|-|-|-|
| `/enemy_tracking/markers` | `visualization_msgs/msg/MarkerArray` | 可视化标记 |

### 4.3 Action 客户端

| Action | 类型 | 描述 |
|-|-|-|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 导航 |

## 5. Launch Arguments

| 参数 | 描述 | 类型 | 默认值 |
|-|-|-|-|
| `target_topic` | 视觉追踪话题 | string | `armor_solver/target` |
| `costmap_topic` | costmap 话题 | string | `/global_costmap/costmap` |
| `map_frame` | 地图坐标系 | string | `map` |
| `attack_radius` | 攻击半径（与敌人保持的距离） | double | `3.0` |
| `num_sectors` | 候选攻击点数量 | int | `36` |
| `cost_threshold` | costmap 代价阈值（小于此值视为可通行） | int | `50` |
| `goal_update_interval` | 目标更新最小间隔（秒） | double | `1.0` |
| `enable_navigation` | 是否启用导航（调试时可关闭） | bool | `true` |
| `tf_tolerance` | TF 时间容忍度 | double | `0.5` |

## 6. 可视化

在 RViz 中添加 `MarkerArray` 显示 `/enemy_tracking/markers`：

- 蓝色球：敌人位置
- 绿色球：最优攻击点
- 红色球：候选攻击点（亮色为可行点，暗色为障碍物上的点）
- 蓝色圆圈：攻击半径

## 7. 工作原理

```
视觉追踪目标 (Target)
        │
        ▼
   TF 转换到 map 坐标系
        │
        ▼
   生成候选攻击点（敌人周围圆形分布）
        │
        ▼
   costmap 过滤（剔除障碍物上的点）
        │
        ▼
   选择最优点（离机器人最近）
        │
        ▼
   计算攻击姿态（朝向敌人）
        │
        ▼
   发送 Nav2 导航目标
```

## 致谢

- [pb2025_sentry_behavior](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior)
- [pb2025_rm_vision](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_rm_vision)
- [Nav2](https://github.com/ros-planning/navigation2)
