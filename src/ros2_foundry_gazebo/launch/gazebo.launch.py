# NOTE: This launch file is now modified to load a plain .urdf file,
# bypassing the xacro processor for this test.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros2_foundry_description = get_package_share_directory('ros2_foundry_description')
    pkg_ros2_foundry_gazebo = get_package_share_directory('ros2_foundry_gazebo')

    # Directly read the monolithic .urdf file
    robot_description_path = os.path.join(pkg_ros2_foundry_description, 'urdf', 'monolithic_bot.urdf')
    with open(robot_description_path, 'r') as f:
        robot_description_content = f.read()
    robot_description = {'robot_description': robot_description_content}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg_ros2_foundry_gazebo, 'worlds', 'factory.world')}.items()
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'foundry_bot'], output='screen')

    node_robot_state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                                      output='screen', parameters=[robot_description])
    
    controller_spawners = [
        "joint_state_broadcaster",
        "diff_drive_controller",
        "arm_controller",
        "gripper_controller"
    ]
    spawner_nodes = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager"],
        )
        for controller in controller_spawners
    ]

    delay_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_entity, on_exit=spawner_nodes,)
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        delay_controller_spawners,
    ])