# Monopedal Jumping Robot : Project Robotic Devops (FRA501)
This project is part of the FRA501 Robotics DevOps course for third-year students at the Institute of Field Robotics (FIBO), King Mongkut’s University of Technology Thonburi (KMUTT) to simulate the work systems of the Monopedal Jumping Robot (BaLegce) with 1 reaction wheel and 2 propellers on Rviz and Gazebo program with ROS2. The simulation will enable the Monopedal Jumping Robot to move in a jumping pattern, maintain its balance, jump in place, and navigate towards predefined destinations.

# Objective
1. To simulate the work system of the Monopedal Jumping Robot (BaLegce) with 1 reaction wheel and 2 propellers on Rviz and Gazebo program with ROS2.
2. To study the process of simulation in Rviz and Gazebo.
3. To investigate movement and motion control, including the motion behavior of the Monopedal Jumping Robot.

# System Overview 
![image](https://github.com/TanawatPawanta/src/assets/119843578/24f40170-1477-4297-b6b8-99b757d27798)

# Monopedal Jumping Robot
![image](https://github.com/TanawatPawanta/BaLEGce/assets/119843578/98602a57-ede2-49ee-b536-92f153c84349)
have 5 parts of model
- body
- leg
- reaction wheel
- propeller 1
- propeller 2

# Package
## balegce
A package for defining various values to be used in visualizing the output in RVIZ. It also serves as the basic default configuration for the model, intended for further utilization in Gazebo simulations
### 1. config
The directory that contains a parameter file that defines the properties of the model. You can adjust the values in the parameter file.
- kinematic_parameter.yaml
The parameters file contains the position and orientation of the coordinated frame in the model, the types of each joint (prismatic, revolute, continuous), lower and upper ranges of motion for each joint, and the velocity and effort of each joint. In the default of this package, I have defined the kinematic parameter values of the model, referring to the model’s frame in SOLIDWORKS.
![image](https://github.com/TanawatPawanta/BaLEGce/assets/119843578/ad54a09c-ffc6-483e-a624-e530f8b81595)
example : 
```
joint_wheel:
  types: revolute
  orientation: 0.0 -1.5705 0.0
  position: -0.0295 0.0 0.050
  lower: -3.141592653589793
  upper: 3.141592653589793
  velocity: 1000.0
  effort: 1000.0
joint_propeller_1:
  types: continuous
  orientation: 0.0 -1.5705 0.0
  position: -0.00063 -0.04668 0.15142
  velocity: 1000.0
  effort: 1000.0
```
- dynamics_parameters.yaml
The parameters file contains physical properties, including mass, center of mass (COM), and inertia for each link in the model. In the default of this package, I have defined the dynamics parameter values of the model, referring to the model in SOLIDWORKS.
example :
![image](https://github.com/TanawatPawanta/BaLEGce/assets/119843578/c8764e67-4f0d-4748-a932-3a0131a094bc)
```
reactionwheel: 
  mass: 0.01371
  com: 0.00 -0.00519 0.00
  inertia:
    xx: 0.00013552
    yy: 0.07074242
    zz: 0.07078874
    xy: 0.0
    xz: 0.0
    yz: 0.0
```
- collision_parameters.yaml
The parameters file includes the position and orientation of the model within the collision section, The collision frame is referenced from the kinematic frame. It is possible to set the size of the collision through the 'size' parameter in the file.

- sensor_parameters.yaml
The parameters file contains information about the position and orientation of the sensor frame. It allows specifying on which link to mount it through the 'link' parameter.

- visual_parameters.yaml
The parameters file includes the position and orientation of the model within the visualization section, referencing the kinematic frame. Additionally, it includes color to be used in visualizing the model.

### 2. robot/visual
In this directory, there is a file named file.xacro was used to construct the robot model.
- robot.xacro
The file will get the path of file.xacro for constructing the robot model.
- properties.xacro
The file will get the parameters from the config file as a variable for constructing the robot model.
- manipulator.xacro
This file will use the variables that are defined in properties.xacro for constructing the visual, inertial, and collision of a link model and joint in Rviz and Gazebo.
- sensor.xacro
This file will use the variables about the sensor that are defined in properties.xacro for constructing the sensor link in the model

### 3. launch
- display.launch.py
This file displays the model which has all links and joints including the sensor link in Rviz. 

## balegce_gazebo
### 1. config
- controller_config.yaml

### 2. robot
- balegce.gazebo.xacro
The file will get the path of file.xacro for constructing the robot model.
- balegce_controller.gazebo.xacro
The file will use the controller plugin from "libgazebo_ros2_control.so" and define where to reference the controller in the model. Including, the force plugin from "libgazebo_ros_force.so" and defining where to reference the force in the model.
- balegce_ground_truth.xacro
The file will use the ground_truth plugin from "libgazebo_ros_p3d.so" and define where to reference the ground_truth in the model. 
- balegce_sensor.gazebo.xacro
The file will use the sensor plugin from "libgazebo_ros_imu_sensor.so" and define where to reference the sensor in the model. 

### 3. scripts
- read_imu.py
This file functions to convert the measured values from the IMU into roll, pitch, and yaw.

### 4. launch
-spawn.launch.py
## balegce_controller
- controller.py

Orientation conrtol diagram
![orientation_control_diagram drawio](https://github.com/TanawatPawanta/BaLEGce/assets/83177015/431b67cc-640c-43f2-bfe0-e7150de67a43)
Contreoller gain can config as launch configulation 


# Installation

1.) Clone the repo to your workspace. You must unzip and put each folder in the workspace.

2.) Place "src" in the workspace

3.) Check in src will have 7 file :
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
ros2 launch balegce spawn.launch.py
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

