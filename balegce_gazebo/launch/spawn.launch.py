from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.event_handlers import OnExecutionComplete, OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler

def generate_launch_description():
    # Launch arguments
    Kp_leg_launch_arg = DeclareLaunchArgument('Kp_leg', default_value='1.0',description="leg's Kp controller gain : float")
    Kp_leg = LaunchConfiguration('Kp_leg')

    Kp_roll_launch_arg = DeclareLaunchArgument('Kp_roll', default_value='3.0',description="roll's Kp controller gain : float")
    Kp_roll = LaunchConfiguration('Kp_roll')

    Kp_pitch_launch_arg = DeclareLaunchArgument('Kp_pitch', default_value='3500.0',description="pitch's Kp controller gain : float")
    Kp_pitch = LaunchConfiguration('Kp_pitch')
    
    Kp_yaw_launch_arg = DeclareLaunchArgument('Kp_yaw', default_value='55.0',description="yaw's Kp controller gain : float")
    Kp_yaw = LaunchConfiguration('Kp_yaw')

    Kd_leg_launch_arg = DeclareLaunchArgument('Kd_leg', default_value='10.0',description="leg's Kd controller gain : float")
    Kd_leg = LaunchConfiguration('Kd_leg')

    Kd_roll_launch_arg = DeclareLaunchArgument('Kd_roll', default_value='3.0',description="roll's Kd controller gain : float")
    Kd_roll = LaunchConfiguration('Kd_roll')

    Kd_pitch_launch_arg = DeclareLaunchArgument('Kd_pitch', default_value='50.0',description="pitch's Kd controller gain : float")
    Kd_pitch = LaunchConfiguration('Kd_pitch')
    
    Kd_yaw_launch_arg = DeclareLaunchArgument('Kd_yaw', default_value='10.0',description="yaw's Kd controller gain : float")
    Kd_yaw = LaunchConfiguration('Kd_yaw')

    forceConstant_launch_arg = DeclareLaunchArgument('forceConstant', default_value='0.0001',description="Force Constance : float")
    forceConstant = LaunchConfiguration('forceConstant')

    Kd_pitch_launch_arg = DeclareLaunchArgument('Kd_pitch', default_value='0.1',description="pitch's Kd controller gain : float")
    Kd_pitch = LaunchConfiguration('Kd_pitch')
    
    Kd_yaw_launch_arg = DeclareLaunchArgument('Kd_yaw', default_value='0.1',description="yaw's Kd controller gain : float")
    Kd_yaw = LaunchConfiguration('Kd_yaw')



    # --|URDF Robot description|--#
    pkg = get_package_share_directory('balegce_gazebo')
    path = os.path.join(pkg,'robot','balegce.gazebo.xacro')
    ros_description = xacro.process_file(path).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description':ros_description}]
    )

    # --|Start Gazebo server and client|--#
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )
    
    # --|Nodes|--#
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

    read_imu = Node(
        package="read_sensor",
        executable="read_imu.py",
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "effort_controllers", "--controller-manager", "controller_manager"]
    )

    controller = Node(
        package = "balegce_controller",
        executable = "controller.py",
        parameters=[
            {'Kp_leg':Kp_leg},
            {'Kp_roll':Kp_roll},
            {'Kp_pitch':Kp_pitch},
            {'Kp_yaw':Kp_yaw},
            {'Kd_leg':Kd_leg},
            {'Kd_roll':Kd_roll},
            {'Kd_pitch':Kd_pitch},
            {'Kd_yaw':Kd_yaw},
            {'forceConstant':forceConstant}
        ]
    )

    event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action = joint_state_broadcaster,
            on_exit=[read_imu, controller_spawner, controller]
        )
    )

    launch_description = LaunchDescription()
    launch_description.add_action(Kp_leg_launch_arg)
    launch_description.add_action(Kp_roll_launch_arg)
    launch_description.add_action(Kp_pitch_launch_arg)
    launch_description.add_action(Kp_yaw_launch_arg)

    launch_description.add_action(Kd_roll_launch_arg)
    launch_description.add_action(Kd_leg_launch_arg)
    launch_description.add_action(Kd_pitch_launch_arg)
    launch_description.add_action(Kd_yaw_launch_arg)
    launch_description.add_action(forceConstant_launch_arg)
    
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(gazebo)
    launch_description.add_action(robot_spawner)
    launch_description.add_action(joint_state_broadcaster)
    launch_description.add_action(event_handler)
    return launch_description