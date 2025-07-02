本项目旨在基于ROS2和Gazebo，从零开始搭建一个集成了轮式底盘与六轴机械臂的移动抓取机器人仿真平台。平台将模拟工业自动化场景，实现从接收顶层任务指令到自主导航、视觉感知、精确抓取与放置的端到端(End-to-End)自动化作业流程。

该项目旨在探索移动操作(Mobile Manipulation)领域的标准工作流与最佳实践。

## 开发路线图与进度 (Development Roadmap & Progress)

### ✅ **阶段一: 环境搭建与基础配置**

**目标:** 准备好一个功能正常的ROS2开发环境，为后续开发奠定基础。

- [x] **系统与ROS2安装**
    
    - [x] 安装 `Ubuntu 22.04 LTS`
        
    - [x] 安装 `ROS2 Humble Hawksbill` 并配置好环境变量
        
    - [x] 安装 `ros-dev-tools` 等核心编译工具
        
- [x] **仿真与IDE环境**
    
    - [x] 安装 `ros-humble-gazebo-ros-pkgs`
        
    - [x] 安装并配置 `VS Code` (C++/Python/ROS插件)
        
- [x] **创建ROS2工作空间**
    
    - [x] 创建 `~/ros2_ws/src` 目录结构
        
    - [x] 成功执行 `colcon build` 初始化工作空间
        
- [x] **成果验证:**
    
    - [x] 终端可正常运行 `ros2` 系列命令
        
    - [x] 可独立启动 `gazebo` 与 `rviz2`
        

### ✅ **阶段二: 机器人模型整合**

**目标:** 创建一个包含轮式底盘和机械臂的统一机器人模型，并在仿真中正确显示。

- [x] **创建描述包 (Description Package)**
    
    - [x] 创建 `mobile_manipulator_description` 功能包
        
    - [x] 在包内创建 `urdf`, `launch`, `rviz`, `meshes` 目录
        
- [x] **整合URDF/xacro**
    
    - [x] 编写顶层 `mobile_manipulator.urdf.xacro` 文件
        
    - [x] 使用 `<xacro:include>` 引入TurtleBot4和Lite6机械臂的xacro文件
        
    - [x] **核心:** 使用 `fixed` 类型的 `<joint>` 将机械臂基座连接到车体上
        
- [x] **创建可视化Launch文件**
    
    - [x] 编写 `display.launch.py`，包含 `robot_state_publisher`, `joint_state_publisher_gui` 和 `rviz2`
        
    - [x] 创建并加载自定义的RViz配置文件 `view_model.rviz`
        
- [x] **成果验证:**
    
    - [x] 运行 `display.launch.py` 后，能在RViz中看到完整的轮臂机器人模型
        
    - [x] 拖动 `joint_state_publisher_gui` 滑块，模型关节能够相应运动
        
    - [x] 在RViz中检查TF树，确认机械臂的坐标系正确地挂载在车体上
        

### ✅ **阶段三: 自主导航 (Nav2)**

**目标:** 让机器人在一个已知的地图中实现高精度的自主导航。

- [x] **创建仿真世界与启动文件**
    
    - [x] 创建一个简单的工厂布局 `.world` 文件
        
    - [x] 编写 `gazebo.launch.py`，用于在指定世界中加载机器人模型
        
- [x] **SLAM建图**
    
    - [x] 编写 `slam.launch.py`，集成Gazebo启动并运行 `slam_toolbox`
        
    - [x] 手动遥控机器人在环境中移动，完成地图构建
        
    - [x] 使用 `nav2_map_server` 保存地图为 `my_map.yaml` 和 `my_map.pgm`
        
- [x] **配置并运行Nav2**
    
    - [x] 创建 `navigation.launch.py`
        
    - [x] 编写 `nav2_params.yaml` 配置文件
        
    - [x] 在Launch文件中调用 `nav2_bringup`，并传入自定义的地图和参数文件
        
- [x] **成果验证:**
    
    - [x] 运行 `navigation.launch.py` 后，RViz中能看到地图、粒子云和代价地图
        
    - [x] 在RViz中点击 `Nav2 Goal`，机器人能够自主规划路径并移动到目标位置
        

### ✅ **阶段四: 运动规划 (MoveIt2)**

**目标:** 实现通过代码精确控制机械臂完成复杂运动。

- [x] **配置MoveIt**
    
    - [x] 运行 `MoveIt Setup Assistant`，加载整合后的URDF模型
        
    - [x] 配置碰撞矩阵、规划组 (`arm`, `gripper`)、预设位姿 (`home`, `ready`)
        
    - [x] 生成 `mobile_manipulator_moveit_config` 功能包
        
- [x] **测试MoveIt配置**
    
    - [x] 运行 `demo.launch.py`
        
    - [x] 在RViz中通过交互标记拖动机械臂末端，验证 `Plan & Execute` 功能
        
- [x] **编写API控制脚本**
    
    - [x] 创建一个Python/C++节点，使用 `moveit_commander` 或 `move_group_interface`
        
    - [x] 编写代码，实现对机械臂到指定位姿或预设位姿的控制
        
- [x] **成果验证:**
    
    - [x] 通过运行脚本，Gazebo中的机械臂能够准确移动到代码中设定的目标位置
        

### ⏳ **阶段五: 视觉感知与抓取集成**

**目标:** 让机器人能“看到”目标物体，并计算出其在世界中的三维坐标。

- [x] **添加并验证相机**
    
    - [x] 在URDF文件中为机器人添加深度相机插件
        
    - [x] 启动仿真，使用 `ros2 topic list` 和 `rqt_image_view` 确认图像话题已发布
        
- [ ] **创建感知节点**
    
    - [ ] 编写 `perception_node.py` 节点
        
    - [ ] **图像处理:** 使用 `cv_bridge` 和 `OpenCV` 识别图像中的目标物体，获取像素坐标 `(u, v)`
        
    - [ ] **3D坐标计算:** 结合深度信息和相机内参，计算物体在相机坐标系下的 `(x, y, z)`
        
    - [ ] **坐标系转换:** 使用 `tf2_ros` 将坐标转换到 `world` 坐标系
        
- [ ] **提供服务接口**
    
    - [ ] 将感知功能封装成一个ROS2 Service，接收请求，返回目标物体的位姿
        
- [ ] **成果验证:**
    
    - [ ] 在Gazebo中放置一个目标方块
        
    - [ ] 调用服务后，能成功返回方块在世界坐标系下的三维坐标，并在RViz中可视化验证
        

### ⏳ **阶段六: 顶层任务调度与总集成**

**目标:** 使用行为树将所有子功能串联起来，实现一键启动的全自动任务。

- [ ] **定义系统接口**
    
    - [ ] 创建自定义Action `PerformTask.action`，用于接收顶层任务指令
        
- [ ] **创建行为树(BT)节点**
    
    - [ ] 编写C++源文件，创建继承自 `BT::RosActionNode` 或 `BT::RosServiceNode` 的自定义节点，用于调用Nav2、MoveIt2和感知服务
        
- [ ] **编写行为树逻辑**
    
    - [ ] 创建 `main_task.xml` 文件，使用XML语法编排完整的任务逻辑
        
- [ ] **创建并运行总控制器**
    
    - [ ] 编写 `bt_runner` 节点，用于加载和执行行为树
        
    - [ ] 编写最终的 `demo.launch.py`，一键启动所有相关节点
        
- [ ] **成果验证:**
    
    - [ ] 运行 `demo.launch.py`
        
    - [ ] 发送一个顶层任务的Action Goal
        
    - [ ] **最终演示:** 机器人完全自主地、按顺序地完成移动、抓取、放置等一系列动作，全程无需人工干预。
