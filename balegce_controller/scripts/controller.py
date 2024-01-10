#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Wrench
from input_interfaces.srv import SetPosition

class controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Create Timer
        self.create_timer(0.01, self.timerCallback)

        # Create publisher
        self.pub_posCommand     = self.create_publisher(Float64MultiArray, "/position_controllers/commands", 10)
        self.pub_veloCommand    = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        self.pub_forceR         = self.create_publisher(Wrench, "/propeller_r/force", 10)
        self.pub_forceL         = self.create_publisher(Wrench, "/propeller_l/force", 10)

        # Create Subscriber
        self.create_subscription(Twist, 'euler_angles', self.orientation_callback, 10)

        # Variables
        self.orientation = [0.0, 0.0, 0.0]

    # Methods ===========================================
    def wrenchPub(self, publisher, force, torque):
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
        # position
        pubPos = Float64MultiArray()
        pubPos.data = [0.0]
        # velocity
        pubVelo = Float64MultiArray()
        pubVelo.data = [0.0, 0.0]
        # publish
        self.pub_posCommand.publish(pubPos)
        self.pub_veloCommand.publish(pubVelo)

    # Subscriber Callback ------------------------
    def orientation_callback(self, msg):
        self.orientation[0] = msg.angular.roll_x
        self.orientation[1] = msg.angular.pitch_y
        self.orientation[2] = msg.angular.yaw_z

    # Controller ---------------------------------
    def controller(self):
        # orientation error
        diff_orient_x = 0 - self.orientation[0]
        diff_orient_y = 0 - self.orientation[1]
        diff_orient_z = 0 - self.orientation[2]
        

def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
