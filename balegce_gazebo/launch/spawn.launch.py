from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler

def generate_launch_description():
    pkg = get_package_share_directory('balegce_gazebo')
    path = os.path.join(pkg,'robot','balegce.gazebo.xacro')
    ros_description = xacro.process_file(path).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description':ros_description}]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )
    robot_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'baLEGce'
            ]
        )
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"]
    )

    velocity_controllers = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["velocity_controllers", "--controller-manager", "controller_manager"]
    )

    launch_description = LaunchDescription()
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(gazebo)
    launch_description.add_action(robot_spawner)
    launch_description.add_action(joint_state_broadcaster)
    launch_description.add_action(velocity_controllers)
    return launch_description