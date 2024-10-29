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
import math
from math import pi
import time
   
class turtle_robot(Node):

    def __init__(self):
        super().__init__('turtle_robot') 
        self.frequency = 0.004
        self.max_velocity = 3

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)        
        #callback_group=self.callBackGrp
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped,'/goal_pose', self.goal_pose_sub_func, qos_profile=10 )
        # self.tilt_msg_sub = self.create_subscription(Tilt,'/tilt_msg', self.tilt_msg_sub_func, qos_profile=10 )
        self.turtle_pose_msg_sub = self.create_subscription(Pose,'/turtle1/pose', self.turtle_pose_sub_func, qos_profile=10 )
        self.goal_pose = PoseStamped()
        # self.tilt_msg = Tilt()
        self.current_pose = Pose()
        self.updated_pose = [0.0,0.0,0.0]
        
        # Transforms:

        # Static broadcasters publish on /tf_static. We will only need to publish this once.
        self.static_broadcaster = StaticTransformBroadcaster(self)
        world_base_tf = TransformStamped()
        world_base_tf.header.stamp = self.get_clock().now().to_msg()
        world_base_tf.header.frame_id = 'world'
        world_base_tf.child_frame_id = 'odom'
        world_base_tf.transform.translation.x = -5.2
        world_base_tf.transform.translation.y = -5.2
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
    
    def goal_pose_sub_func(self,msg):
        self.goal_pose = msg
        # self.get_logger().info(f"The brick transform is: x: {self.goal_pose.pose.position.x} y: {self.goal_pose.pose.position.y} and z: {self.goal_pose.pose.position.z}")
        # Update vel
        # till reached goal tollerance          
        dist = math.sqrt( (msg.pose.position.y - self.current_pose.y) **2 +(msg.pose.position.x - self.current_pose.x ) **2 )
        if (dist <= 0.4):
            self.get_logger().info("Stopping the platform !")
            linear_x = 0.0
            linear_y = 0.0
        else:
            self.get_logger().info("Moving the platform [Turtle-node] !")
            theta = math.atan( (msg.pose.position.y - self.current_pose.y) / (msg.pose.position.x - self.current_pose.x ) ) 
            linear_x = self.max_velocity * math.cos(theta)
            linear_y = self.max_velocity * math.sin(theta)
        # Cmd pub
        cmd_pub_msg = Twist()
        cmd_pub_msg.linear = Vector3 ( x = float(linear_x), y= float(linear_y), z= 0.0)
        self.cmd_publisher.publish(cmd_pub_msg)
    
    
    def tilt_msg_sub_func(self,msg):
        self.tilt_msg = msg

    def turtle_pose_sub_func(self,msg):
        self.current_pose = msg
        # Dyn Transform
        base = TransformStamped()
        base.header.frame_id = 'odom'
        base.child_frame_id = 'base_link'
        time = self.get_clock().now().to_msg()
        base.header.stamp = time
        # Applying the robot motion here.
        base.transform.translation = Vector3 ( x = float(msg.x), y = float(msg.y), z= 0.0) 
        self.broadcaster.sendTransform(base)
        # Odom publisher
        odom_pub_msg = Odometry()
        odom_pub_msg.pose.pose.position.x = msg.x + 5.2
        odom_pub_msg.pose.pose.position.y = msg.y + 5.2
        odom_pub_msg.header.stamp = self.get_clock().now().to_msg()
        odom_pub_msg.header.frame_id = 'odom'
        odom_pub_msg.child_frame_id = 'base_link'
        self.odom_publisher.publish(odom_pub_msg)


    def calcute_updated_vel(self, pose):
        # initial pose of robot.
        updated_pose = [0.0,0.0,0.0]
        # till reached goal tollerance  
        if (self.goal_pose.pose.position.x != 0 and self.goal_pose.pose.position.y != 0 and self.goal_pose.pose.position.z != 0 ):            
            dt = self.frequency
            # 5.2 is the odom - world frame
            theta = math.atan( (self.goal_pose.pose.position.y + 5.2) / (self.goal_pose.pose.position.x + 5.2) ) 
            updated_pose[0] = pose[0] + self.max_velocity * math.cos(theta) * dt
            updated_pose[1] = pose[1] + self.max_velocity * math.sin(theta) * dt
            # self.get_logger().info(f"The brick transform is: x: {self.goal_pose.pose.position.x} y: {self.goal_pose.pose.position.y} and z: {self.goal_pose.pose.position.z}")
        return updated_pose     
 

    def timer_callback(self):
        # JSP
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_positions[1] = 0.02 
        theta_dummy = 0.02
        # wheel rotation here.        
        self.joint_positions[3] = theta_dummy
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

