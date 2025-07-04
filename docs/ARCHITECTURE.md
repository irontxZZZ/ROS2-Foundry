# 移动抓取机器人仿真平台 - 系统架构设计

**版本:** 1.0
**创建日期:** 2025-07-04
**负责人:** SA (System Architect)
**关联需求:** [docs/CAPABILITY_MATRIX.md](CAPABILITY_MATRIX.md)

---

## 1. 架构概述

本项目采用基于ROS2 Humble Hawksbill的分布式、模块化软件架构。系统以**行为树 (Behavior Tree)** 为顶层任务调度核心，集成**自主导航 (Nav2)**、**运动规划 (MoveIt2)**、**视觉感知 (Perception)** 和 **硬件接口 (ros2_control)** 四大核心子系统，实现端到端的自动化移动抓取任务。

整体架构遵循分层设计思想，将任务层、功能层与执行层解耦，以提高系统的可扩展性、可维护性和可复用性。

## 2. 系统组件图

```mermaid
graph TD
    subgraph "任务层 (Task Layer)"
        FMS_Interface[外部任务接口<br>(fms_interface)];
        Task_Coordinator[任务协调器<br>(task_coordinator/BehaviorTree)];
    end

    subgraph "功能层 (Functional Layer)"
        Navigation[导航系统<br>(Nav2)];
        Manipulation[操作规划系统<br>(MoveIt2)];
        Perception[视觉感知系统<br>(perception_sim)];
    end

    subgraph "执行层 (Execution Layer)"
        ROS2_Control[硬件抽象层<br>(ros2_control)];
        Robot_Description[机器人模型<br>(mobile_manipulator_description)];
    end

    subgraph "仿真/物理世界"
        Gazebo[Gazebo仿真环境];
        RealRobot[真实机器人硬件];
    end

    FMS_Interface -- "RequestTask.action" --> Task_Coordinator;
    Task_Coordinator -- "调用BT节点" --> Navigation;
    Task_Coordinator -- "调用BT节点" --> Manipulation;
    Task_Coordinator -- "调用BT节点" --> Perception;

    Perception -- "GetObjectPose.srv" --> Task_Coordinator;
    Navigation -- "cmd_vel" --> ROS2_Control;
    Manipulation -- "joint_trajectory" --> ROS2_Control;

    ROS2_Control -- "驱动指令" --> Gazebo;
    ROS2_Control -- "驱动指令" --> RealRobot;
    Gazebo -- "传感器数据" --> Perception;
    Gazebo -- "传感器数据" --> Navigation;
    Robot_Description -- "URDF" --> Gazebo;
    Robot_Description -- "URDF" --> Navigation;
    Robot_Description -- "URDF" --> Manipulation;

end
```

## 3. 核心子系统详述

### 3.1. 任务协调器 (Task Coordinator)
- **核心技术:** BehaviorTree.CPP v4
- **职责:**
    - 接收来自`fms_interface`的顶层任务请求 (`RequestTask.action`)。
    - 加载并执行定义在XML文件中的行为树逻辑。
    - 作为客户端，调用导航、操作和感知系统提供的ROS2服务和动作。
- **关键节点:** `MapsToPose`, `PickObject`, `PlaceObject`, `GetObjectPose`。

### 3.2. 自主导航 (Nav2)
- **核心技术:** ROS2 Navigation 2 Stack
- **职责:**
    - **定位:** 使用AMCL算法在预先构建的地图中进行机器人定位。
    - **路径规划:** 根据目标点生成全局和局部路径。
    - **障碍物规避:** 实时躲避动态和静态障碍物。
- **接口:**
    - **Action:** `nav2_msgs/action/NavigateToPose`
    - **Topic (out):** `/cmd_vel` (速度指令)
    - **Topic (in):** `/scan` (激光雷达), `/odom` (里程计)

### 3.3. 运动规划 (MoveIt2)
- **核心技术:** ROS2 MoveIt 2
- **职责:**
    - 为六轴机械臂提供运动学求解。
    - 生成无碰撞的关节空间轨迹。
    - 提供高级规划能力，如MoveIt Task Constructor (MTC)用于复杂的抓取序列。
- **接口:**
    - **Action:** `moveit_msgs/action/MoveGroup`
    - **Service:** `moveit_msgs/srv/GetMotionPlan`

### 3.4. 视觉感知 (Perception)
- **核心技术:** OpenCV, `cv_bridge`, `tf2_ros`
- **职责:**
    - 处理来自仿真相机（或真实相机）的RGB和深度图像。
    - 识别场景中的目标物体。
    - 计算并提供目标物体在`world`坐标系下的三维位姿。
- **接口:**
    - **Service:** `perception_sim/srv/GetObjectPose`
    - **Topic (in):** `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info`

## 4. 接口定义 (Interface Control Document)

系统的核心接口在第一阶段被定义和冻结，以确保各模块并行开发的兼容性。

- **`fms_interface/action/RequestTask.action`**:
  - **用途:** 接收外部系统（如车队管理系统）下发的宏任务指令。
  - **阶段:** 阶段1 (SA)
  - **状态:** 已定义

- **`perception_sim/srv/GetObjectPose.srv`**:
  - **用途:** 请求感知系统返回指定物体的三维位姿。
  - **阶段:** 阶段1 (SA)
  - **状态:** 已定义

## 5. 配置管理

- **机器人模型:** 所有子系统共享同一个URDF模型 (`mobile_manipulator.urdf.xacro`)，由`robot_state_publisher`统一发布TF树。
- **参数文件:** 各个子系统（Nav2, MoveIt2, ros2_control）的配置参数存储在`config/`目录下，并通过Launch文件在启动时加载。