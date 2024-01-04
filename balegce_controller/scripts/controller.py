#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Wrench

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
        self.orientation    = [0.0, 0.0, 0.0]
        self.referenceAngles = [0.0, 0.0, 0.0]
        self.Kp_wheel       = 1.0
        self.Kp_propellerL  = 1.0
        self.Kp_propellerR  = 1.0
        self.forceConstance = 1.0

    # Methods ===========================================
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
        pubPos.data = [0.0]
        # velocity
        pubVelo = Float64MultiArray()
        # wheel prop1(left) prop2(right) leg
        pubVelo.data = [controller_output[0], controller_output[1], controller_output[2], 0.0]
        # publish
        self.wrenchPub(self.pub_forceL, force=[0.0, 0.0, -controller_output[3]], torque=[0.0, 0.0, 0.0])
        self.wrenchPub(self.pub_forceR, force=[0.0, 0.0, -controller_output[4]], torque=[0.0, 0.0, 0.0])
        self.pub_posCommand.publish(pubPos)
        self.pub_veloCommand.publish(pubVelo)

    # Subscriber Callback ------------------------
    def orientation_callback(self, msg):
        self.orientation[0] = msg.angular.x
        self.orientation[1] = msg.angular.y
        self.orientation[2] = msg.angular.z

    # Controller ---------------------------------
    def velocityController(self)->list[float]:
        # orientation error
        diff_orient_x = self.referenceAngles[0] - self.orientation[0]
        diff_orient_y = self.referenceAngles[1] - self.orientation[1]
        diff_orient_z = self.referenceAngles[2] - self.orientation[2]
        # controller
        wheel_velo = self.Kp_wheel*diff_orient_x 
        propellerR_velo = self.Kp_propellerR*diff_orient_y + self.Kp_propellerR*diff_orient_z
        propellerL_velo = self.Kp_propellerL*diff_orient_y - self.Kp_propellerL*diff_orient_z
        # velocityToforce
        propellerR_force = self.forceConstance*propellerR_velo
        propellerL_force = self.forceConstance*propellerL_velo
        output = [wheel_velo, propellerL_velo, propellerR_velo, propellerL_force, propellerR_force]
        return output


def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
