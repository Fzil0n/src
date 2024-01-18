#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Wrench
import math

class leg_controller(Node):
    def __init__(self):
        super().__init__('leg_controller')

        #--|Create Timer|--#
        self.create_timer(0.01, self.timerCallback)

        #--|Create publisher|--#
        self.pub_effCommand     = self.create_publisher(Float64MultiArray, "/effort_controllers/commands", 10)
        
        #--|Create Subscriber|--#
        self.sub_contact = self.create_subscription(ContactsState,"/contact",self.contact_callback,10)
        
        #--|Variables|--#
        self.trigger = False


    # Methods ===========================================
    # Timer Callback -----------------------------
    def timerCallback(self):
        pubEff = Float64MultiArray() 
        if self.trigger:
            pubEff.data = [-100.0]
        else:
            pubEff.data = [100.0]
        self.pub_effCommand.publish(pubEff)

    # Subscriber Callback ------------------------
    def contact_callback(self, msg):
        if len(msg.states) != 0: 
            self.trigger = True
        else:
            self.trigger = False

    # Publish Function ---------------------------
    def effort_pub(self, publisher, force:list[float], torque:list[float])->None:
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

def main(args=None):
    rclpy.init(args=args)
    node = leg_controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
