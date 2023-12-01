from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os, yaml
import xacro
# def visualizer(package:str,rviz_file:str):
#     pkg = get_package_share_directory(package)
#     rviz_path = os.path.join(pkg,'config',rviz_file)
    
#     rviz = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz',
#         arguments=['-d', rviz_path],
#         output='screen')
#     return rviz
def generate_launch_description():
    pkg = get_package_share_directory('motor_control')
    path = os.path.join(pkg,'robot','visual/robot.xacro')
    with open (path,'r') as file:
        robot_desc = file.read()
    robot_descripton = xacro.process_file(path).toxml()
    robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description':robot_descripton}]
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen')
    
    joint_state_publisher_gui = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui')
    
    launch_description = LaunchDescription()
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(rviz)
    launch_description.add_action(joint_state_publisher_gui)


    return launch_description