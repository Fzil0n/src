#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist,Quaternion,Point
import math
import numpy as np
from input_interfaces.srv import SetPosition,SetOrientation

class interface_node(Node):
    # ========Constructor===========
    def __init__(self):
        super().__init__('interface_node')
        self.pose = [0.0,0.0,0.0]
        self.orientation = [0.0,0.0,0.0,0.0]
        self.create_service(SetPosition, "interface/Target_pose", self.Target_pose_callback)
        self.create_service(SetOrientation, "interface/Target_angle", self.Target_orientation_callback)

    # ========Class's methods=========
        
    def Target_pose_callback(self, request, responce):
        self.pose = [request.target.x,request.target.y]
        responce.result = True
        return responce

    def Target_orientation_callback(self, request, responce):
        self.orientation = [request.orien.x,request.orien.y,request.orien.z,request.orien.w]
        responce.result = True
        return responce

def main(args=None):
    rclpy.init(args=args)
    node = interface_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
