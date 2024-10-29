#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from geometry_msgs.msg import Twist, Vector3, Point, PoseStamped
import time

class robotState(Enum):
    WAIT = 0,
    MOVE = 1,
    CATCHED = 2,
    TILT = 3,
    SLIPPED = 4
   
class catcher(Node):

    def __init__(self):
        super().__init__('catcher') 
        self.frequency = 0.04
        self.updated_pose = [0.0,0.0,0.0]

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


    # Todo: 1. make the robot move in straight line.
    # 2. theta  = omega cal
    # 3. cal error 
    # 4. stem movement.

    def get_brick_transform(self):
        
        try:
            transform = self.brick_tf_buffer.lookup_transform('world', 'brick', rclpy.time.Time())
            self.get_logger().info(f"The brick transform is: x: {transform.transform.translation.x} y: {transform.transform.translation.y} and z: {transform.transform.translation.z}")
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {'world'} to {'brick'}: {ex}')
            transform = [0.0 , 0.0, 0.0]       
        return transform

    def get_platform_transform(self):
        try:
            transform = self.platform_tf_buffer.lookup_transform('world', 'platform', rclpy.time.Time())
            self.get_logger().info(f"The platform transform is: x: {transform.transform.translation.x} y: {transform.transform.translation.y} and z: {transform.transform.translation.z}")
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {'world'} to {'platform'}: {ex}')
            transform = [0.0 , 0.0, 0.0]        
        return transform
    
    def get_brick_platform_transform(self):
        try:
            transform = self.platform_brick_tf_buffer.lookup_transform('platform', 'brick', rclpy.time.Time())
            self.get_logger().info(f"The platform-brick transform is: x: {transform.transform.translation.x} y: {transform.transform.translation.y} and z: {transform.transform.translation.z}")
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {'platform'} to {'brick'}: {ex}')
            transform = [0.0 , 0.0, 0.0]        
        return transform


    def timer_callback(self):
        self.get_logger().info("publish speed" ,once=True)
        brick_transform = self.get_brick_transform()
        platform_transform = self.get_platform_transform()
        brick_platform_transform = self.get_brick_platform_transform()


        
        
    

def main(args=None):
    rclpy.init(args=args)
    node = catcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)         