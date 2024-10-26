#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Point
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from .quaternion import angle_axis_to_quaternion
from math import pi
import time


class turtle_robot(Node):

    def __init__(self):
        super().__init__('turtle_robot') 
        self.frequency = 0.5
        self.reset_future = 0
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(self.frequency, self.timer_callback)
        #callback_group=self.callBackGrp

        # Transforms:

        # Static broadcasters publish on /tf_static. We will only need to publish this once.
        self.static_broadcaster = StaticTransformBroadcaster(self)
        world_base_tf = TransformStamped()
        world_base_tf.header.stamp = self.get_clock().now().to_msg()
        world_base_tf.header.frame_id = 'world'
        world_base_tf.child_frame_id = 'odom'
        world_base_tf.transform.translation.x = -5.0
        world_base_tf.transform.translation.y = -5.0
        self.static_broadcaster.sendTransform(world_base_tf)
        self.get_logger().info('Static Transform: world->odom')

        # Dynamic broadcaster publishes on /tf.
        self.broadcaster = TransformBroadcaster(self)
        self.dx = 10 
        
        # Joint states
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
        # Dyn Transform
        base = TransformStamped()
        base.header.frame_id = 'odom'
        base.child_frame_id = 'base_link'
        # Need to capture the robot 
        base.transform.translation.x = -float(self.dx)


        time = self.get_clock().now().to_msg()
        base.header.stamp = time
        self.broadcaster.sendTransform(base)

        # Update the movement
        self.dx -= 1
        if self.dx == 0:
            self.dx = 10

        # JSP
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
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

