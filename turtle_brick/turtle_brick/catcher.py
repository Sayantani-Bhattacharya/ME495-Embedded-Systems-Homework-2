#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, Vector3, Point, PoseStamped
import time


class catcher(Node):

    def __init__(self):
        super().__init__('catcher') 
        self.frequency = 0.01
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)      
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.updated_pose = [0.0,0.0,0.0]

        # self.brick_listner = self.create_subscription( , '/tf', 10) #brick drop listner
        # platform tf lister

        
        # Dynamic broadcaster publishes on /tf.
        self.broadcaster = TransformBroadcaster(self)

        # Joint states
        self.joint_state = JointState()
        self.joint_state.name = ["j0","j1","j2","j3"]
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]

        self.start_time = time.time()
        self.timer = self.create_timer(self.frequency, self.timer_callback)

    def publish_joint_states(self):
        self.joint_state_publisher.publish(self.joint_state)
        wheel_theta = 0.0
        self.joint_positions[3] = float(wheel_theta)
        self.joint_state.position = self.joint_positions
        self.joint_state_publisher.publish(self.joint_state)

    # Todo: 1. make the robot move in straight line.
    # 2. theta  = omega cal
    # 3. cal error 
    # 4. stem movement.

    def timer_callback(self):
        self.get_logger().info("publish speed")







def main(args=None):
    rclpy.init(args=args)
    node = catcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)    