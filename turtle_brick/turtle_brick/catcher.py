#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from geometry_msgs.msg import Twist, Vector3, Point, PoseStamped
import math
import time

class robotState(Enum):
    WAIT = 0,
    MOVE = 1,
    WAITING_FOR_BRICK = 2,
    CATCHED = 3,
    TILT = 4,
    SLIPPED = 5
   
class catcher(Node):

    def __init__(self):
        super().__init__('catcher')
        self.frequency = 0.04
        self.updated_pose = [0.0,0.0,0.0]
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', qos_profile=10)
        self.robot_state = robotState.WAIT
        self.init_flag = 1 
        self.brick_exists = False
        self.init_brick_pose = 1000

        # Initialize TF buffers and listeners
        self.brick_tf_buffer = Buffer()
        self.brick_tf_listner = TransformListener(buffer=self.brick_tf_buffer, node=self)
        self.brick_tf_broadcaster = TransformBroadcaster(self)

        self.platform_tf_buffer = Buffer()
        self.platform_tf_listner = TransformListener(buffer=self.platform_tf_buffer, node=self)
        self.platform_broadcaster = TransformBroadcaster(self)

        self.platform_brick_tf_buffer = Buffer()
        self.platform_brick_tf_listner = TransformListener(buffer=self.platform_brick_tf_buffer, node=self)
        self.platform_brick_tf_broadcaster = TransformBroadcaster(self)

        self.start_time = time.time()
        self.timer = self.create_timer(self.frequency, self.timer_callback)

    def get_brick_transform(self):        
        try:
            transform = self.brick_tf_buffer.lookup_transform('world', 'brick', rclpy.time.Time())
            self.brick_exists = True
            # self.get_logger().info(f"The brick transform is: x: {transform.transform.translation.x} y: {transform.transform.translation.y} and z: {transform.transform.translation.z}")
        except TransformException as ex:
            return 
            
        # Cal the initial pose of brick
        if (self.init_flag and self.brick_exists):
            # if (brick_exists):  
                self.init_brick_pose = transform.transform.translation.z   
                self.init_flag = 0           
        return transform

    def get_platform_transform(self):
        try:
            transform = self.platform_tf_buffer.lookup_transform('world', 'platform', rclpy.time.Time())                        
        except TransformException as ex:
            # pass
            return
        return transform
    
    def get_brick_platform_transform(self):
        try:
            transform = self.platform_brick_tf_buffer.lookup_transform('platform', 'brick', rclpy.time.Time())
        except TransformException as ex:
            return
            # transform = [-1.0 , -1.0, -1.0]       
        return transform

    def timer_callback(self):
        self.get_logger().info("publish speed" ,once=True)
        brick_transform = self.get_brick_transform()
        platform_transform = self.get_platform_transform()
        brick_platform_transform = self.get_brick_platform_transform()

        # To detect drop:   
        # self.get_logger().info(f"brick 1 :: {self.brick_exists}")      
        if (self.brick_exists and self.robot_state == robotState.WAIT):
            self.currZ = brick_transform.transform.translation.z
            if (self.init_brick_pose - self.currZ >= 0.09):
                self.get_logger().info("Started to Drop !", once=True )
                self.robot_state = robotState.MOVE

        # Publish the goal pose.
        if (self.robot_state == robotState.MOVE):
            goal_pose_pub_msg = PoseStamped()
            goal_pose_pub_msg.header.stamp = self.get_clock().now().to_msg()
            dist = math.sqrt( brick_platform_transform.transform.translation.y **2 + brick_platform_transform.transform.translation.x **2 )
            if (dist <= 0.4):
                self.robot_state = robotState.WAITING_FOR_BRICK
                self.get_logger().info("Catched [Platform motion] !", once=True )
                goal_pose_pub_msg.pose.position.x = brick_transform.transform.translation.x
                goal_pose_pub_msg.pose.position.y = brick_transform.transform.translation.y
                self.goal_pose_pub.publish(goal_pose_pub_msg) 

            # Stick brick to robot.
            elif (brick_transform != [-1.0 , -1.0, -1.0] and self.robot_state != robotState.CATCHED):
                self.get_logger().info("About to Catch!", once=True )
                goal_pose_pub_msg.pose.position.x = brick_transform.transform.translation.x
                goal_pose_pub_msg.pose.position.y = brick_transform.transform.translation.y
                goal_pose_pub_msg.pose.position.z = brick_transform.transform.translation.z
                self.goal_pose_pub.publish(goal_pose_pub_msg) 
                
        elif(self.robot_state == robotState.WAITING_FOR_BRICK):            
            if (brick_transform.transform.translation.z <= 0.2):
                self.get_logger().info(" Catched !", once=True )
                self.robot_state = robotState.CATCHED
            else:
                goal_pose_pub_msg = PoseStamped()
                goal_pose_pub_msg.header.stamp = self.get_clock().now().to_msg()
                self.get_logger().info(" Waiting for brick to drop [Platform motion] Platform stopped !", once=True )
                goal_pose_pub_msg.pose.position.x = brick_transform.transform.translation.x
                goal_pose_pub_msg.pose.position.y = brick_transform.transform.translation.y
                self.goal_pose_pub.publish(goal_pose_pub_msg)


def main(args=None):
    rclpy.init(args=args)
    node = catcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)         