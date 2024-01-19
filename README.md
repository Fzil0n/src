# Monopedal Jumping Robot : Project Robotic Devops (FRA501)
This project is part of the FRA501 Robotics DevOps course for third-year students at the Institute of Field Robotics (FIBO), King Mongkut’s University of Technology Thonburi (KMUTT) to simulate the work systems of the Monopedal Jumping Robot (BaLegce) with 1 reaction wheel and 2 propellers on Rviz and Gazebo program with ROS2. The simulation will enable the Monopedal Jumping Robot to move in a jumping pattern, maintain its balance, jump in place, and navigate towards predefined destinations.

# Objective
1. To simulate the work system of the Monopedal Jumping Robot (BaLegce) with 1 reaction wheel and 2 propellers on Rviz and Gazebo program with ROS2.
2. To study the process of simulation in Rviz and Gazebo.
3. To investigate movement and motion control, including the motion behavior of the Monopedal Jumping Robot.

# System Overview 
![image](https://github.com/TanawatPawanta/src/assets/119843578/24f40170-1477-4297-b6b8-99b757d27798)

# Monopedal Jumping Robot
![image](https://github.com/TanawatPawanta/BaLEGce/assets/119843578/f78eff2a-5bc5-424f-b59f-905da474b976)
The orientation of the Monopedal Robot is defined using ZXY Euler angles. The leg mechanism is positioned in the X-Z plane, with the leg's extension-contraction aligned along the Z-axis, intersecting the Center of Gravity (CG) of the robot. The Reaction Wheels in the tail are parallel to the Y-axis, enabling control of the robot's pitch. Additionally, Thrusters are pointing in the +Y direction, oriented upwards in the +Z direction. They are symmetrically positioned along the Z-axis on the plane parallel to the X-axis. The combined force from both thrusters generates a moment along the X-axis, allowing control of the Roll. Meanwhile, the difference in force produces a moment along the Z-axis, enabling control of the Yaw. With these features, the robot can move freely in all six axes.

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
- spawn.launch.py
This file displays(spawn) the model which has all links and joints including the sensor link and shows behavior of movement in Gazebo.

### 5. worlds
- balegce_world.world
Is a file that describes the gazebo’s world properties such as Sun movement and Friction. In this project, use friction between ground_plane and link of model.
```
<friction>
  <ode>
    <mu>3000</mu>
    <mu2>3000</mu2>
  </ode>
  <torsional>
    <ode/>
  </torsional>
</friction>
```

## balegce_controller
### 1. scripts
- controller.py
Orientation conrtol diagram
![orientation_control_diagram drawio](https://github.com/TanawatPawanta/BaLEGce/assets/83177015/431b67cc-640c-43f2-bfe0-e7150de67a43)

Contreoller gain can config as launch configulation 
- leg_controller.py
This file check leg contect with ground and control leg of the model

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

# Testing out BaLEGce (monopodal jumping robot) in **Rviz**
- Terminal 1: Run launch file in terminal
```
ros2 launch balegce display.launch.py
```

# Testing out BaLEGce (monopodal jumping robot) in **Gazebo**
- Terminal 1: Run launch file in terminal
```
ros2 launch balegce_gazebo spawn.launch.py
```


# Schematics of System
![image](https://github.com/TanawatPawanta/BaLEGce/assets/119843578/0af87688-bae7-4fd0-9f2f-c765c9003816)

# Node
- /contact_plugin
- /controller
- /controller_manager
- /effort_controllers
- /gazebo
- /gazebo_ros2_control_plugin
- /imu_plugin
- /joint_body_state_publisher
- /joint_propeller_1_state_publisher
- /joint_propeller_2_state_publisher
- /joint_state_broadcaster
- /joint_wheel_state_publisher
- /leg_controller
- /propeller_l/gazebo_ros_force
- /propeller_r/gazebo_ros_force
- /read_imu_node
- /robot_state_publisher
- /velocity_controllers

# Our Team
- Napassorn Techasombooranakit 64340500035
- Wasupol Hengsritawat 64340500049
- Tanawat Pawanta 64340500061

