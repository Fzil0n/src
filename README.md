# Monopedal Jumping Robot : Project Robotic Devops (FRA501)
โปรเจคนี้เป็นโปรเจคสำหรับจัดทำ Simulation ของ Monopedal Jumping Robot เพื่อสังเกตลักษณะและพฤติกรรมการเคลื่อนที่ 
และเพื่อเรียนรู้และศึกษาการทำ Simulation ใน Rviz และ Gazebo โดยทำการ Simulation ให้ Monopedal Jumping Robot 
สามารถเคลื่อนที่ในรูปแบบของการกระโดด และทำการทรงตัว เพื่อให้สามารถกระโดดอยู่กับที่ และทำการกระโดดไปยังจุดหมายที่ได้กำหนดไว้ได้

# Objective
1. เพื่อศึกษาการทำ Simulation ใน Rviz และ Gazebo
2. เพื่อศึกษาการเคลื่อนที่ และการควบคุมการเคลื่อนที รวมถึงพฤติกรรมการเคลื่อนที่ของ Monopedal Jumping Robot
3. เพื่อศึกษาพื้นฐานการใช้ ROS2

# System Overview 
![image](https://github.com/TanawatPawanta/src/assets/119843578/24f40170-1477-4297-b6b8-99b757d27798)

# Installation

1.) Clone the repo to your workspace. You must unzip and put each folder in the workspace.

2.) place "src" in workspace

3.) check in src will have 3 file : "turtlesim_plus_control","turtlesim_plus_control_interfaces" and "turtlesim_plus"

4.) Build "src" in your workspace.
```
cd ~/[your_workspace]
colcon build 
source install/setup.bash
```

# Testing out turtlesim_control
- Terminal 1: Run launch file in terminal
```
ros2 launch turtlesim_plus_control following.launch.py
```

# Schematics of System
![Untitled Diagram drawio](https://github.com/TanawatPawanta/src/assets/119843578/6c28736e-a969-4754-8d2b-3aaeefbd6f7b)
- Command Line Interface : Fill in the input, which consists of the angle and distance.
## Package
- Input Interface : Package to enable input from the command line
- Orientation Controller : Package for controlling the orientation of a robot.
- Leg Length Controller : Package for controlling the Leg Length of a robot.
- Gazebo : Simulation Movement

# Node

# Service

