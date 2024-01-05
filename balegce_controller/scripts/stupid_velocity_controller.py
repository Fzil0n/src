#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class stupid_velocity_controller_node(Node):
    def __init__(self):
        super().__init__('stupid_velocity_controller_node')
        # create publisher
        self.pub_veloCommand = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        self.create_timer(0.01, self.timerCallback)
    # class's methods
    def timerCallback(self):
        pubVelo = Float64MultiArray()
        pubVelo.data = [0.0, 5.5, 0.1, 0.1]
        self.pub_veloCommand.publish(pubVelo)


def main(args=None):
    rclpy.init(args=args)
    node = stupid_velocity_controller_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
