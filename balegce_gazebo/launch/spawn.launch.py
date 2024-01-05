from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler

def generate_launch_description():
    # Launch arguments
    Kp_wheel_launch_arg = DeclareLaunchArgument('Kp_wheel', default_value='1.0',description="Wheel joint controller gain : float")
    Kp_wheel = LaunchConfiguration('Kp_wheel')

    Kp_propellerL_launch_arg = DeclareLaunchArgument('Kp_propellerL', default_value='1.0',description="Propeller left joint controller gain : float")
    Kp_propellerL = LaunchConfiguration('Kp_propellerL')
    
    Kp_propellerR_launch_arg = DeclareLaunchArgument('Kp_propellerR', default_value='1.0',description="Propeller right joint controller gain : float")
    Kp_propellerR = LaunchConfiguration('Kp_propellerR')

    forceConstance_launch_arg = DeclareLaunchArgument('forceConstance', default_value='1.0',description="Force Constance : float")
    forceConstance = LaunchConfiguration('forceConstance')

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
            '-entity', 'baLEGce',
            '-z','0.145'
            ]
        )
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"]
    )

    euler_angle_imu = Node(
        package="read_sensor",
        executable="read_imu.py",
    )

    # position_controllers = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     output="screen",
    #     arguments=["position_controllers", "--controller-manager", "controller_manager"]
    # )

    velocity_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "controller_manager"]
    )

    # controller = Node(
    #     package = "balegce_controller",
    #     executable = "controller.py",
    #     parameters=[
    #         {'Kp_wheel':Kp_wheel},
    #         {'Kp_propellerL':Kp_propellerL},
    #         {'Kp_propellerR':Kp_propellerR},
    #         {'forceConstance':forceConstance}
    #     ]
    # )

    launch_description = LaunchDescription()
    launch_description.add_action(Kp_wheel_launch_arg)
    launch_description.add_action(Kp_propellerL_launch_arg)
    launch_description.add_action(Kp_propellerR_launch_arg)
    launch_description.add_action(forceConstance_launch_arg)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(gazebo)
    launch_description.add_action(robot_spawner)
    launch_description.add_action(joint_state_broadcaster)
    launch_description.add_action(euler_angle_imu)
    # launch_description.add_action(position_controllers)
    launch_description.add_action(velocity_controllers)
    # launch_description.add_action(controller)
    return launch_description