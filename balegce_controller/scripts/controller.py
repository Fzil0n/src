#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import Twist, Wrench
from sensor_msgs.msg import Imu, JointState
import math

class controller(Node):
    def __init__(self):
        super().__init__('controller')

        #--|Create Timer|--#
        self.create_timer(0.01, self.timerCallback)

        #--|Create publisher|--#
        # self.pub_posCommand     = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self.pub_veloCommand    = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        self.pub_effCommand     = self.create_publisher(Float64MultiArray, "/effort_controllers/commands", 10)
        self.pub_forceR         = self.create_publisher(Wrench, "/propeller_r/force", 10)
        self.pub_forceL         = self.create_publisher(Wrench, "/propeller_l/force", 10)
        self.pub_orien_error    = self.create_publisher(Float64MultiArray, "/orien_error", 10)
        self.pub_velo_error     = self.create_publisher(Float64MultiArray, "/velo_error", 10)
        
        #--|Create Subscriber|--#
        self.create_subscription(Twist, 'euler_angles', self.curr_orientation_callback, 10)
        self.sub_imu = self.create_subscription(Imu,"/imu",self.imu_callback,10)
        self.sub_angularAcc = self.create_subscription(Float64MultiArray,"angularAccelaration",self.angularAcc_callback, 10)
        # self.sub_joint_body_states = self.create_subscription(JointState,"/joint_body_states",self.sub_sub_joint_body_states_callback,10)

        #--|ROS Parameters|--#
        # Kp controller gain
        self.declare_parameter('Kp_leg',0.0)
        self.declare_parameter('Kp_roll',0.0)
        self.declare_parameter('Kp_pitch',0.0)
        self.declare_parameter('Kp_yaw',0.0)
        # Kd controller gain
        self.declare_parameter('Kd_leg',0.0)
        self.declare_parameter('Kd_roll',0.0)
        self.declare_parameter('Kd_pitch',0.0)
        self.declare_parameter('Kd_yaw',0.0)

        self.declare_parameter('forceConstant',1.0) # thrust gain
        #--|Variables|--#
        self.curr_angularVelocity = [0.0, 0.0, 0.0]  # current angular velocity of robot
        self.curr_angularAccelration = [0.0, 0.0, 0.0]  # current angular acceleration of robot
        self.curr_orientation    = [0.0, 0.0, 0.0]   # current curr_orientation of the robot(roll pitch yaw)
        self.curr_legPosition = 0.0
        self.curr_legVelocity = 0.0
        self.referenceAngles = [0.0, 0.0, 0.0]  # reference curr_orientation of the robot(roll pitch yaw)
        self.referenceOmega = [0.0, 0.0, 0.0]  
        self.referenceLegPosition = 0.0
        self.threshold_orien = 0.02 #1.4591559 degrees
        self.threshold_velo = 0.002

    # Methods ===========================================
    # def sub_sub_joint_body_states_callback(self,msg):
    #     self.curr_legPosition = msg.position[0]
    #     self.curr_legVelocity = msg.velocity[0]
        
    def angularAcc_callback(self,msg):
        self.curr_angularAccelration[0] = msg.data[0]
        self.curr_angularAccelration[1] = msg.data[1]
        self.curr_angularAccelration[2] = msg.data[2]

    def imu_callback(self, msg):
        self.curr_angularVelocity[0] = msg.angular_velocity.x
        self.curr_angularVelocity[1] = msg.angular_velocity.y
        self.curr_angularVelocity[2] = msg.angular_velocity.z

    def wrenchPub(self, publisher, force:list[float], torque:list[float])->None:
        msg = Wrench()
        # force assignment
        msg.force.x = force[0]
        msg.force.y = force[1]
        msg.force.z = force[2]
        # torque assignment
        msg.torque.x = torque[0]
        msg.torque.y = torque[1]
        msg.torque.z = torque[2]
        # publish
        publisher.publish(msg)
    
    # Timer Callback -----------------------------
    def timerCallback(self):
        controller_output = self.velocityController()
        # --position--
        # pubPos = Float64MultiArray()
        # pubPos.data = [self.referenceLegPosition]
        # --velocity--
        pubVelo = Float64MultiArray() 
        pubVelo.data = controller_output   # wheel prop1(left) prop2(right)
        # --generate trust from velocity--
        propellerL_force = self.trustGenerator(speed=controller_output[1], forceConstant=self.get_parameter('forceConstant').value)
        propellerR_force = self.trustGenerator(speed=controller_output[2], forceConstant=self.get_parameter('forceConstant').value)
        # --publish--
        self.wrenchPub(self.pub_forceL, force=[0.0, 0.0, -propellerL_force], torque=[0.0, 0.0, 0.0])
        self.wrenchPub(self.pub_forceR, force=[0.0, 0.0, -propellerR_force], torque=[0.0, 0.0, 0.0])
        # self.pub_posCommand.publish(pubPos)
        self.pub_veloCommand.publish(pubVelo)
        pass

    def trustGenerator(self, speed, forceConstant):
        return forceConstant*speed*speed
    
    # Subscriber Callback ------------------------
    def curr_orientation_callback(self, msg):
        self.curr_orientation[0] = msg.angular.x
        self.curr_orientation[1] = msg.angular.y
        self.curr_orientation[2] = msg.angular.z

    # Controller ---------------------------------
    def roll_PDcontroller(self,error:float, error_dot:float, threshold:float)->float:
        if(abs(error) >= threshold):
            Kp_roll    = self.get_parameter('Kp_roll').value
            Kd_roll    = self.get_parameter('Kd_roll').value
            out = Kp_roll*error + Kd_roll*error_dot
        else:
            out = 0.0
        return out
    
    def pitch_PDcontroller(self, error:float, error_dot:float, threshold: float)->float:
        if(abs(error) >= threshold):
            Kp_pitch    = self.get_parameter('Kp_pitch').value
            Kd_pitch    = self.get_parameter('Kd_pitch').value
            out = Kp_pitch*error + Kd_pitch*error_dot
        else:
            out = 0.0
        return out
    
    def yaw_PDcontroller(self, error:float, error_dot:float, threshold:float)->float:
        if(abs(error) >= threshold):
            Kp_yaw      = self.get_parameter('Kp_yaw').value
            Kd_yaw      = self.get_parameter('Kd_yaw').value
            out = Kp_yaw*error + Kd_yaw*error_dot
        else:
            out =  0.0
        return out
    
    def propeller_velocity_PDController(self, error_pitch:float, error_yaw:float, error_pitch_dot:float, error_yaw_dot:float)->list[float]:
        pitch_command   = self.pitch_PDcontroller(error_pitch, error_pitch_dot, self.threshold_velo)
        yaw_command     = self.yaw_PDcontroller(error_yaw, error_yaw_dot, self.threshold_orien)
        propellerR_velo = pitch_command + yaw_command  
        propellerL_velo = pitch_command - yaw_command
        return [propellerL_velo, propellerR_velo]
    
    def orien_error_pub(self,error_orien_roll:float, error_orien_pitch:float, error_orien_yaw:float)->None:
        pub_orien_error = Float64MultiArray()
        pub_orien_error.data = [error_orien_roll, error_orien_pitch, error_orien_yaw]
        self.pub_orien_error.publish(pub_orien_error)

    def velo_error_pub(self,error_velo_roll:float, error_velo_pitch:float, error_velo_yaw:float)->None:
        pub_velo_error = Float64MultiArray()
        pub_velo_error.data = [error_velo_roll, error_velo_pitch, error_velo_yaw]
        self.pub_velo_error.publish(pub_velo_error)   

    def velocityController(self)->list[float]:
        error_orien_roll = self.referenceAngles[0] - self.curr_orientation[0]
        error_orien_pitch = self.referenceAngles[1] - self.curr_orientation[1]
        error_orien_yaw   = self.referenceAngles[2] - self.curr_orientation[2]

        error_velo_roll = self.referenceOmega[0] - self.curr_angularVelocity[0]
        error_velo_pitch = self.referenceOmega[1] - self.curr_angularVelocity[1]
        error_velo_yaw = self.referenceOmega[2] - self.curr_angularVelocity[2]
        
        # publish error for debugging
        self.orien_error_pub(error_orien_roll, error_orien_pitch, error_orien_yaw)
        self.velo_error_pub(error_velo_roll, error_velo_pitch, error_velo_yaw)

        wheel_velo =  self.roll_PDcontroller(error=error_orien_roll, error_dot=-self.curr_angularVelocity[0], threshold=self.threshold_orien)
        propeller_velo = self.propeller_velocity_PDController(error_velo_pitch, error_orien_yaw, -self.curr_angularAccelration[1], -self.curr_angularVelocity[2])    
        output = [-wheel_velo, -propeller_velo[0], propeller_velo[1]]
        return output
        

def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
