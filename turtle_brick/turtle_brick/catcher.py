#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from geometry_msgs.msg import Twist, Vector3, Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from turtle_brick_interfaces.srv import Tilt
import math
import time

class robotState(Enum):
    WAIT = 0,
    MOVE = 1,
    WAITING_FOR_BRICK = 2,
    CATCHED = 3,
    GOING_TO_HOME = 4,
    TILT = 5,
    SLIPPED = 6
   
class catcher(Node):

    def __init__(self):
        super().__init__('catcher')
        self.frequency = 0.04
        self.updated_pose = [0.0,0.0,0.0]
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', qos_profile=10)
        self.text_marker_pub = self.create_publisher(Marker, 'text_marker', 10)
        self.tilt_pub = self.create_publisher(Empty, '/tilt_msg', 10)

        self.achivability = True
        self.robot_state = robotState.WAIT
        self.init_flag = 1 
        self.brick_exists = False
        self.init_brick_pose = 1000
        self.max_velocity = 6.0
        self.check_once_achivability = False

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
        return transform
    
    def unachivable_pub(self, transform):
        if(transform):
            # Assuming gravity to be 9.8
            h = transform.transform.translation.z
            if h < 0 :
                h = -1 * h
            time_needed_by_brick = math.sqrt(2*h /  9.8)

            dist = math.sqrt(transform.transform.translation.x**2 + transform.transform.translation.y**2)
            time_needed_by_bot = dist / self.max_velocity
            error_tollerance = 0.2

            if (not self.check_once_achivability):
                self.check_once_achivability = True
                if (time_needed_by_brick <= time_needed_by_bot + error_tollerance):
                    self.achivability = False
            
            if (not self.achivability):
                marker_msg = Marker()        
                marker_msg.type = Marker.TEXT_VIEW_FACING
                marker_msg.id = 8
                marker_msg.header.frame_id = 'world'
                marker_msg.header.stamp = self.get_clock().now().to_msg()
                # Set the position of the text marker
                marker_msg.pose.position = Point(x=1.0, y=1.0, z=3.0)
                marker_msg.pose.orientation.w = 1.0
                marker_msg.text = "Unachievable"
                marker_msg.scale.z = 0.80
                marker_msg.color.r = 1.0
                marker_msg.color.g = 1.0
                marker_msg.color.b = 1.0
                marker_msg.color.a = 1.0
                marker_msg.lifetime.sec = 0
                # Publish the marker
                self.text_marker_pub.publish(marker_msg)
                self.get_logger().info("Published text marker")
            else:
                self.get_logger().info("Achivable!!", once=True)

    def timer_callback(self):
        self.get_logger().info("publish" ,once=True)
        brick_transform = self.get_brick_transform()
        platform_transform = self.get_platform_transform()
        brick_platform_transform = self.get_brick_platform_transform()
        self.unachivable_pub(brick_platform_transform)

        if (self.achivability):

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
                self.get_logger().info(f" H val [Platform motion] {brick_transform.transform.translation.z} !", once = True )
                # 1.8 : brick and platform height : need to parameterise it.
                if (brick_transform.transform.translation.z <= 1.9):
                    self.get_logger().info(" Catched !", once=True )
                    self.robot_state = robotState.CATCHED
                else:
                    goal_pose_pub_msg = PoseStamped()
                    goal_pose_pub_msg.header.stamp = self.get_clock().now().to_msg()
                    self.get_logger().info(" Waiting for brick to drop [Platform motion] Platform stopped !", once=True )
                    goal_pose_pub_msg.pose.position.x = brick_transform.transform.translation.x
                    goal_pose_pub_msg.pose.position.y = brick_transform.transform.translation.y
                    self.goal_pose_pub.publish(goal_pose_pub_msg)

            elif(self.robot_state == robotState.CATCHED ):
                self.get_logger().info(" At Catched case [catcher node]!", once=True )
                self.robot_state = robotState.TILT

            # Going to home has an issue with movement direction.
            elif (self.robot_state == robotState.GOING_TO_HOME):
                self.get_logger().info(" Bot going to home [catcher node]!", once=True )
                goal_pose_pub_msg = PoseStamped()
                goal_pose_pub_msg.header.stamp = self.get_clock().now().to_msg()
                goal_pose_pub_msg.pose.position.x = 2.0
                goal_pose_pub_msg.pose.position.y = 2.0
                self.goal_pose_pub.publish(goal_pose_pub_msg)
                dist_to_home = math.sqrt(platform_transform.transform.translation.x**2  +  platform_transform.transform.translation.y**2)
                if (dist_to_home < 0.4):
                    self.get_logger().info("Bot reached Home [catcher node]!",once=True )
                    self.robot_state = robotState.TILT

            elif(self.robot_state == robotState.TILT):
                    self.get_logger().info("Tilting now [catcher node]!",once=True )
                    tilt_trigger_msg = Empty()
                    # publish tilt.
                    self.tilt_pub.publish(tilt_trigger_msg)

def main(args=None):
    rclpy.init(args=args)
    node = catcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)         