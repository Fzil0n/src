#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Wrench
from sensor_msgs.msg import Imu, JointState

class prismatic_controller(Node):
    def __init__(self):
        super().__init__('prismatic_controller')

        #--|Create Timer|--#
        self.create_timer(0.01, self.timerCallback)

        #--|Create publisher|--#
        self.pub_posCommand     = self.create_publisher(Float64MultiArray, "/position_controllers/commands", 10)

    # Methods ===========================================
    # Timer Callback -----------------------------
    def timerCallback(self):
        pubPos = Float64MultiArray()
        pubPos.data = [0.2]
        self.pub_posCommand.publish(pubPos)

def main(args=None):
    rclpy.init(args=args)
    node = prismatic_controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
