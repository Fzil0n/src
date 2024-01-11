#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist,Quaternion,Point
import math
import numpy as np
from std_msgs.msg import Bool
from input_interfaces.srv import SetPosition
from input_interfaces.srv import SetOrientation

class interface_node(Node):
    # ========Constructor===========
    def __init__(self):
        super().__init__('interface_node')
        self.pose = [0.0,0.0]
        self.orientation = [0.0,0.0,0.0,0.0]
        self.controller_enable = False
        self.create_service(SetPosition, "interface/Target_pose", self.Target_pose_callback)
        self.create_service(SetOrientation, "interface/Target_angle", self.Target_orientation_callback)

    # ========Class's methods=========
    def Target_pose_callback(self, request, responce):
        self.target_pos = [request.position.x, request.position.y]
        self.controller_enable = True
        return SetPosition.Response()

    def Target_orientation_callback(self, request, responce):
        self.orientation = [request.orientation.x,request.orientation.y,request.orientation.z,request.orientation.w]
        self.controller_enable = True
        return SetOrientation.Response()

def main(args=None):
    rclpy.init(args=args)
    node = interface_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
