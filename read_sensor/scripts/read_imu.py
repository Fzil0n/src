#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float64MultiArray

class DummyNode(Node):
    # ========Constructor===========
    def __init__(self):
        super().__init__('read_imu_node')
        self.eulerAngles = Twist()
        self.quanternion_read = [0.0, 0.0, 0.0, 0.0]
        self.curr_angularVelocity = [0.0, 0.0, 0.0]
        self.last_angularVelocity   = [0.0, 0.0, 0.0]
        self.dt = 0.001
        # create subscriber
        self.sub_imu = self.create_subscription(Imu,"/imu",self.imu_callback,10)
        # create publisher
        self.pub_euler = self.create_publisher(Twist,"euler_angles",10)
        self.pub_angularAccelaration = self.create_publisher(Float64MultiArray,"angularAccelaration",10)
        # create timer
        self.create_timer(self.dt,self.timeCallback)

    #========Class's methods=========
    def imu_callback(self,msg):
        self.curr_angularVelocity[0] = msg.angular_velocity.x
        self.curr_angularVelocity[1] = msg.angular_velocity.y
        self.curr_angularVelocity[2] = msg.angular_velocity.z

        self.quanternion_read[0] = msg.orientation.x
        self.quanternion_read[1] = msg.orientation.y
        self.quanternion_read[2] = msg.orientation.z
        self.quanternion_read[3] = msg.orientation.w

    def timeCallback(self):
        self.quaternion_to_euler()
        self.angularAccelaration_pub(self.angularAceleration_cal())

    def angularAceleration_cal(self)->list[float]:
        angularAcc = [0.0, 0.0, 0.0]
        for i in range(3):
            angularAcc[i] = (self.curr_angularVelocity[i] - self.last_angularVelocity[i])/self.dt
            self.last_angularVelocity[i] = self.curr_angularVelocity[i]
        return angularAcc
    
    def angularAccelaration_pub(self, angularAcc:list[float])->None:
        pub_data = Float64MultiArray()
        pub_data.data = [angularAcc[0], angularAcc[1], angularAcc[2]]
        self.pub_angularAccelaration.publish(pub_data)

    def quaternion_to_euler(self):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).

        Parameters:
        - q: a 4-element list or numpy array representing the quaternion (x, y, z, w).

        Returns:
        - None
        """
        x, y, z, w = self.quanternion_read

        # roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x**2 + y**2)
        roll_x = np.arctan2(t0, t1)

        # pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = np.where(t2 > 1.0, 1.0, t2)
        t2 = np.where(t2 < -1.0, -1.0, t2)
        pitch_y = np.arcsin(t2)

        # yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y**2 + z**2)
        yaw_z = np.arctan2(t3, t4)

        self.eulerAngles.angular.x = roll_x
        self.eulerAngles.angular.y = pitch_y
        self.eulerAngles.angular.z = yaw_z   
        self.pub_euler.publish(self.eulerAngles) 
        
    
def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
