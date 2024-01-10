# Monopedal Jumping Robot : Project Robotic Devops (FRA501)
This project is part of the FRA501 Robotics DevOps course for third-year students at the Institute of Field Robotics (FIBO), King Mongkut’s University of Technology Thonburi (KMUTT) to simulate the work systems of the Monopedal Jumping Robot (BaLegce) with 1 reaction wheel and 2 propellers on Rviz and Gazebo program with ROS2. The simulation will enable the Monopedal Jumping Robot to move in a jumping pattern, maintain its balance, jump in place, and navigate towards predefined destinations.

# Objective
1. To simulate the work system of the Monopedal Jumping Robot (BaLegce) with 1 reaction wheel and 2 propellers on Rviz and Gazebo program with ROS2.
2. To study the process of simulation in Rviz and Gazebo.
3. To investigate movement and motion control, including the motion behavior of the Monopedal Jumping Robot.

# System Overview 
![image](https://github.com/TanawatPawanta/src/assets/119843578/24f40170-1477-4297-b6b8-99b757d27798)

# Installation

1.) Clone the repo to your workspace. You must unzip and put each folder in the workspace.

2.) place "src" in workspace

3.) check in src will have 7 file :
  - balegce
  - balegce_controller
  - balegce_gazebo
  - input
  - input_interfaces
  - moter_control
  - read_sensor

4.) Build "src" in your workspace.
```
cd ~/[your_workspace]
colcon build 
source install/setup.bash
```

# Testing out turtlesim_control in **Rviz**
- Terminal 1: Run launch file in terminal
```
ros2 launch balegce display.launch.py
```

# Testing out turtlesim_control in **Gazebo**
- Terminal 1: Run launch file in terminal
```
ros2 launch balegce display.launch.py
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

