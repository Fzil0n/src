#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

<<<<<<< HEAD
class stupid_velocity_controller(Node):
=======
class stupid_velocity_controller_node(Node):
>>>>>>> main
    def __init__(self):
        super().__init__('stupid_velocity_controller')
        # create publisher
        self.pub_veloCommand = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        self.create_timer(0.01, self.timerCallback)
<<<<<<< HEAD
    # methods
    def timerCallback(self):
        pubVelo = Float64MultiArray()
        pubVelo.data = [0.0]
=======
    # class's methods
    def timerCallback(self):
        pubVelo = Float64MultiArray()
        pubVelo.data = [0.0, 5.5, 0.1, 0.1]
>>>>>>> main
        self.pub_veloCommand.publish(pubVelo)


def main(args=None):
    rclpy.init(args=args)
<<<<<<< HEAD
    node = stupid_velocity_controller()
=======
    node = stupid_velocity_controller_node()
>>>>>>> main
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
