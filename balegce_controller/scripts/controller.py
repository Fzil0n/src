#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Wrench
from sensor_msgs.msg import Imu, JointState

class controller(Node):
    def __init__(self):
        super().__init__('controller')

        #--|Create Timer|--#
        self.create_timer(0.01, self.timerCallback)

        #--|Create publisher|--#
        # self.pub_posCommand     = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self.pub_veloCommand    = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        self.pub_forceR         = self.create_publisher(Wrench, "/propeller_r/force", 10)
        self.pub_forceL         = self.create_publisher(Wrench, "/propeller_l/force", 10)

        #--|Create Subscriber|--#
        self.create_subscription(Twist, 'euler_angles', self.curr_orientation_callback, 10)
        self.sub_imu = self.create_subscription(Imu,"/imu",self.imu_callback,10)
        # self.sub_joint_body_states = self.create_subscription(JointState,"/joint_body_states",self.sub_sub_joint_body_states_callback,10)

        #--|ROS Parameters|--#
        # Kp controller gain
        self.declare_parameter('Kp_leg',1.0)
        self.declare_parameter('Kp_wheel',1.0)
        self.declare_parameter('Kp_pitch',1.0)
        self.declare_parameter('Kp_yaw',1.0)
        # Kd controller gain
        self.declare_parameter('Kd_leg',0.1)
        self.declare_parameter('Kd_pitch',0.1)
        self.declare_parameter('Kd_yaw',0.1)
        self.declare_parameter('forceConstant',1.0) # thrust gain
        #--|Variables|--#
        self.curr_angularVelocity = [0.0, 0.0, 0.0]  # current angular velocity of robot
        self.curr_orientation    = [0.0, 0.0, 0.0]   # current curr_orientation of the robot(roll pitch yaw)
        self.curr_legPosition = 0.0
        self.curr_legVelocity = 0.0
        self.referenceAngles = [0.0, 0.0, 0.0]  # reference curr_orientation of the robot(roll pitch yaw)
        self.referenceLegPosition = 0.0
        self.threshole = 0.0002 #0.0114591559 degrees

    # Methods ===========================================
    # def sub_sub_joint_body_states_callback(self,msg):
    #     self.curr_legPosition = msg.position[0]
    #     self.curr_legVelocity = msg.velocity[0]

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
        # position
        pubPos = Float64MultiArray()
        pubPos.data = [self.referenceLegPosition]
        # velocity
        pubVelo = Float64MultiArray() 
        pubVelo.data = controller_output   # leg(body) wheel prop1(left) prop2(right)
        # generate trust from velocity
        propellerL_force = self.trustGenerator(speed=controller_output[1], forceConstant=self.get_parameter('forceConstant').value)
        propellerR_force = self.trustGenerator(speed=controller_output[2], forceConstant=self.get_parameter('forceConstant').value)
        # publish
        self.wrenchPub(self.pub_forceL, force=[0.0, 0.0, -propellerL_force], torque=[0.0, 0.0, 0.0])
        self.wrenchPub(self.pub_forceR, force=[0.0, 0.0, -propellerR_force], torque=[0.0, 0.0, 0.0])
        # self.pub_posCommand.publish(pubPos)
        self.pub_veloCommand.publish(pubVelo)

    def trustGenerator(self, speed, forceConstant):
        return forceConstant*speed*speed
    
    # Subscriber Callback ------------------------
    def curr_orientation_callback(self, msg):
        self.curr_orientation[0] = msg.angular.x
        self.curr_orientation[1] = msg.angular.y
        self.curr_orientation[2] = msg.angular.z

    # Controller ---------------------------------
    def pitchController(self, error:float)->float:
        if(error >= self.threshole):
            Kp_pitch    = self.get_parameter('Kp_pitch').value
            Kd_pitch    = self.get_parameter('Kd_pitch').value
            out = Kp_pitch*error + Kd_pitch*self.curr_angularVelocity[1]
        else:
            out = 0.0
        return out
    
    def yawController(self, error:float)->float:
        if(error >= self.threshole):
            Kp_yaw      = self.get_parameter('Kp_yaw').value
            Kd_yaw      = self.get_parameter('Kd_yaw').value
            out =  Kp_yaw*error + Kd_yaw*self.curr_angularVelocity[2]
        else:
            out =  0.0
        return out
        
    def propeller_velocityController(self, error_pitch:float, error_yaw:float)->list[float]:
        pitch_command   = self.pitchController(error_pitch)
        yaw_command     = self.yawController(error_yaw)
        propellerR_velo = pitch_command + yaw_command  
        propellerL_velo = pitch_command - yaw_command
        return [propellerL_velo, propellerR_velo]
    
    def velocityController(self)->list[float]:
        # Kp_leg      = self.get_parameter('Kp_leg').value
        Kp_wheel    = self.get_parameter('Kp_wheel').value
        # Kd_leg      = self.get_parameter('Kd_leg').value
        # error
        # diff_leg      = self.referenceLegPosition - self.curr_legPosition
        diff_orient_x = self.referenceAngles[0] - self.curr_orientation[0]
        diff_orient_y = self.referenceAngles[1] - self.curr_orientation[1]
        diff_orient_z = self.referenceAngles[2] - self.curr_orientation[2]
        # controllers
        # leg_velo = Kp_leg*diff_leg + Kd_leg*self.curr_legVelocity
        wheel_velo = Kp_wheel*diff_orient_x 
        propeller_velo = self.propeller_velocityController(diff_orient_y, diff_orient_z)
        output = [wheel_velo, propeller_velo[0], propeller_velo[1]]
        return output

def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
