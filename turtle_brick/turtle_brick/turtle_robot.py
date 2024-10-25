#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Point
from sensor_msgs.msg import JointState
import time


class turtle_robot(Node):

    def __init__(self):
        super().__init__('turtle_robot') 
        self.frequency = 0.5
        self.reset_future = 0
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(self.frequency, self.timer_callback)
        #callback_group=self.callBackGrp
        
        self.joint_state = JointState()
        self.joint_state.name = ["j0","j1","j2","j3"]
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.start_time = time.time() 

    def publish_joint_states(self):
        self.joint_state_publisher.publish(self.joint_state)
        self.joint_positions[1] = 0.5
        self.joint_state.position = self.joint_positions
        self.joint_state_publisher.publish(self.joint_state)

    def timer_callback(self):
        self.joint_positions[1] = 0.02 
        self.joint_state.position = self.joint_positions
        self.joint_state_publisher.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = turtle_robot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)


# Broadcasting the location of the base_link frame relative to the odom frame

