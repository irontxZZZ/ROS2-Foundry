# 自动化零件配套机器人仿真系统 (ros2_foundry)

本项目旨在构建一个基于ROS2 Humble的移动操作机器人仿真系统，完整实现"自动化零件配套"的工业应用场景。

## 项目愿景

通过该项目，全面展示ROS2全栈开发能力，包括：
- **轮臂机器人任务开发**：实现复杂的抓取、分拣与放置。
- **仿真与实机迁移**：在仿真环境中充分验证，为未来迁移到物理机器人奠定基础。
- **自主导航**：在模拟的工厂环境中实现鲁棒的自主定位与导航。
- **系统集成**：与模拟的工厂管理系统(FMS)进行通信，接收任务并反馈结果。

## 项目文档与日志

* [系统架构](docs/architecture.md)
* [开发日志](docs/DevelopLog.md)
* [调试日志](docs/DebugLog.md)

## 技术栈

| 组件         | 技术选型/版本             |
|--------------|---------------------------|
| **ROS** | ROS2 Humble Hawksbill (LTS) |
| **仿真器** | Gazebo Fortress           |
| **导航** | Nav2 (1.1.12)             |
| **操作** | MoveIt2 + MTC (2.5.4)     |
| **任务编排** | BehaviorTree.CPP (4.3.7)  |
| **硬件抽象** | ros2_control (2.33.0)     |
| **开发语言** | C++17 / Python 3          |

## 项目结构

```
ros2_foundry/
├── .gitignore               # Git忽略配置
├── docs/                    # 项目文档
├── launch/                  # 顶层启动文件
├── README.md                # 你正在阅读的这个文件
├── scripts/                 # 辅助脚本
└── src/                     # ROS2 功能包源码
    ├── ros2_foundry_bringup       # 顶层集成与任务编排(BT)
    ├── ros2_foundry_description   # 机器人模型(URDF)
    ├── ros2_foundry_gazebo        # Gazebo仿真环境
    ├── ros2_foundry_interfaces    # 自定义消息/服务/动作
    ├── ros2_foundry_moveit_config # MoveIt2配置
    ├── ros2_foundry_navigation    # Nav2导航配置
    ├── ros2_foundry_perception    # 视觉感知
    └── ros2_foundry_bt_nodes      # 自定义行为树节点
```

## 安装与构建
(稍后补充)

## 使用方法
(稍后补充)
