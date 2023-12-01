#!/usr/bin/python3
from balegce_gazebo.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import numpy as np

class DummyNode(Node):
    # ========Constructor===========
    def __init__(self):
        super().__init__('read_imu_node')
        self.eulerAngles = Twist()
        self.quanternion_read = [0, 0, 0, 0]
        # create subscriber
        self.sub_imu = self.create_subscription(Imu,"/imu",self.imu_callback,10)
        # create publisher
        self.pub_euler = self.create_publisher(Twist,"euler_angles",10)
        # create timer
        self.create_timer(0.001,self.timeCallback)

    #========Class's methods=========
    def imu_callback(self,msg):
        self.quanternion_read[0] = msg.orientation.x
        self.quanternion_read[1] = msg.orientation.y
        self.quanternion_read[2] = msg.orientation.z
        self.quanternion_read[3] = msg.orientation.w
    def timeCallback(self):
           self.quaternion_to_euler()
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
