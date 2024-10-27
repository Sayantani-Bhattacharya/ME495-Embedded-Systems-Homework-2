#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Point, PoseStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from .quaternion import angle_axis_to_quaternion
from turtle_brick_interfaces.srv import Tilt
from turtlesim.msg import Pose
from math import pi
import time


class turtle_robot(Node):

    def __init__(self):
        super().__init__('turtle_robot') 
        self.frequency = 0.01
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)        
        #callback_group=self.callBackGrp
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.goal_pose_sub = self.create_subscription(PoseStamped,'/goal_pose', self.goal_pose_sub_func, qos_profile=10 )
        # self.tilt_msg_sub = self.create_subscription(Tilt,'/tilt_msg', self.tilt_msg_sub_func, qos_profile=10 )
        self.turtle_pose_msg_sub = self.create_subscription(Pose,'/turtle1/pose', self.turtle_pose_sub_func, qos_profile=10 )

        self.goal_pose = PoseStamped()
        # self.tilt_msg = Tilt()
        self.turtle_pose_msg = Pose()
        

        # Transforms:

        # Static broadcasters publish on /tf_static. We will only need to publish this once.
        self.static_broadcaster = StaticTransformBroadcaster(self)
        world_base_tf = TransformStamped()
        world_base_tf.header.stamp = self.get_clock().now().to_msg()
        world_base_tf.header.frame_id = 'world'
        world_base_tf.child_frame_id = 'odom'
        world_base_tf.transform.translation.x = -5.0
        world_base_tf.transform.translation.y = -5.0
        # height of half box, full stem, 2 radius.: set this from yaml later.
        wheel_radius = 0.3
        platform_ground_height = 1.8
        world_base_tf.transform.translation.z = (platform_ground_height - 2*wheel_radius) * 0.15 + (platform_ground_height - 2*wheel_radius) * 0.1 + 2*wheel_radius
        self.static_broadcaster.sendTransform(world_base_tf)
        self.get_logger().info('Static Transform: world->odom')

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
        self.joint_positions[1] = 0.5
        self.joint_state.position = self.joint_positions
        self.joint_state_publisher.publish(self.joint_state)
    
    def goal_pose_sub_func(self,msg):
        self.goal_pose = msg

    def tilt_msg_sub_func(self,msg):
        self.tilt_msg = msg

    def turtle_pose_sub_func(self,msg):
        self.turtle_pose_msg = msg
        # 'x': 'float',
        # 'y': 'float',
        # 'theta': 'float',
        # 'linear_velocity': 'float',
        # 'angular_velocity': 'float'

    def timer_callback(self):
        # Dyn Transform
        base = TransformStamped()
        base.header.frame_id = 'odom'
        base.child_frame_id = 'base_link'
        time = self.get_clock().now().to_msg()
        base.header.stamp = time
        # self.get_logger().info('ffffffffffffffffffffff111111111111ffff Transform: world->odom')


        # Applying the robot motion here.
        base.transform.translation = Vector3 ( x = float(self.turtle_pose_msg.x), y = float(self.turtle_pose_msg.y), z=0.0) 
        self.broadcaster.sendTransform(base)
        # self.get_logger().info('ffffffffffffffffffffff111111111111ffff Transform: world->odom')


        # JSP
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_positions[1] = 0.02 
        # wheel rotation here
        theta_dummy = 0.02
        self.joint_positions[3] = theta_dummy
        self.joint_state.position = self.joint_positions
        self.joint_state_publisher.publish(self.joint_state)

        # Cmd pub
        cmd_pub_msg = Twist()
        cmd_pub_msg.linear = Vector3 ( x = float(self.turtle_pose_msg.x), y= float(self.turtle_pose_msg.y), z= 0.0)
        cmd_pub_msg.angular = Vector3 ( x = 0.0, y= 0.0, z= self.turtle_pose_msg.angular_velocity )
        self.cmd_publisher.publish(cmd_pub_msg)

        # # Odom publisher
        # odom_pub_msg = Odometry()
        # odom_pose = Point()
        # odom_pose.x = 1
        # odom_pose.y = 2
        # odom_pose.z = 1
        # odom_pub_msg.pose.pose.position(odom_pose)  # # (odom_pose)   ->> prob is here !!
        # odom_pub_msg.header.stamp = self.get_clock().now().to_msg()
        # odom_pub_msg.header.frame_id = 'odom'
        # odom_pub_msg.child_frame_id = 'base_link'
        # # Twist same as cmd_vel
        # # odom_pub_msg.twist.twist.linear = Vector3(x=self.linear_velocity, y=0.0, z=0.0)
        # # odom_pub_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.angular_velocity)
        # self.odom_publisher.publish(odom_pub_msg)

    

def main(args=None):
    rclpy.init(args=args)
    node = turtle_robot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)

