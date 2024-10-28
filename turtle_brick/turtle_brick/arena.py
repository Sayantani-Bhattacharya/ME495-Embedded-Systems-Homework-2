#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from builtin_interfaces.msg import Duration
from turtle_brick.physics import World
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3
from turtle_brick_interfaces.srv import Place, Gravity
import time

class arena(Node):

    def __init__(self):
        super().__init__('arena') 
        self.frequency = 0.004
        self.place = self.create_service(Place, "place", self.place_func) 
        self.drop = self.create_service(Gravity, "drop", self.drop_func)  
        self.brick = Brick()         

        # Marker Array: Walls
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub1 = self.create_publisher(MarkerArray, 'visualization_marker_array', markerQoS)
        self.m_array = MarkerArray()
        height = 1.5        
        thickness = 0.20
        pose1 = (-5.5 + thickness/2, 0.0, height/2)
        pose2 = (5.5 - thickness/2, 0.0, height/2)
        pose3 = (0.0, 5.5 - thickness/2, height/2 )
        pose4 = (0.0, - 5.5 + thickness/2,height/2)
        self.m_array.markers.append( self.create_wall_marker(0,pose1 ) )
        self.m_array.markers.append( self.create_wall_marker(1,pose2 ) )
        self.m_array.markers.append( self.create_wall_marker(2,pose3,length = 0.2, thickness= 11.0, height=1.5 ) )
        self.m_array.markers.append( self.create_wall_marker(3,pose4,length = 0.2, thickness= 11.0, height=1.5 ) )
        self.pub1.publish(self.m_array)

        # Dynamic broadcaster publishes on /tf.
        self.broadcaster = TransformBroadcaster(self)

        # Marker: Brick
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub5 = self.create_publisher(Marker, 'visualization_marker', markerQoS)
        brick_marker = Marker()
        self.timer = self.create_timer(self.frequency, self.timer_callback)

    def create_wall_marker(self,id, position, length = 11.0, thickness=0.20, height=1.5):
        wall_1 = Marker()
        wall_1.header.frame_id = 'world'
        wall_1.header.stamp = self.get_clock().now().to_msg()
        wall_1.lifetime = Duration(sec=0, nanosec=0) 
        wall_1.ns = "Walls"
        wall_1.id = id
        wall_1.type = Marker.CUBE
        wall_1.action = Marker.ADD
        wall_1.scale.x = thickness
        wall_1.scale.y = length
        wall_1.scale.z = height
        wall_1.pose.orientation.x = 0.0
        wall_1.pose.orientation.y = 0.0
        wall_1.pose.orientation.z = 0.0
        wall_1.pose.orientation.w = 0.0
        wall_1.color.r = 0.615
        wall_1.color.g = 0.20
        wall_1.color.b = 1.0
        wall_1.color.a = 0.70
        wall_1.pose.position.x = position[0]
        wall_1.pose.position.y = position[1]
        wall_1.pose.position.z = position[2]
        return(wall_1)
    
    def brick_marker_define(self, pose):
        brick_marker = Marker()          
        brick_marker.header.frame_id = 'world'
        brick_marker.header.stamp = self.get_clock().now().to_msg()
        brick_marker.lifetime = Duration(sec=0, nanosec=0) 
        brick_marker.id = 5
        brick_marker.type = Marker.CUBE
        brick_marker.action = Marker.ADD
        height = 0.20
        length = 0.50
        thickness = 0.20
        brick_marker.scale.x = length
        brick_marker.scale.y = thickness
        brick_marker.scale.z = height

        brick_marker.pose.position.x = pose[0]
        brick_marker.pose.position.y = pose[1]
        brick_marker.pose.position.z = pose[2]

        brick_marker.color.r = 0.80
        brick_marker.color.g = 0.522
        brick_marker.color.b = 0.349
        brick_marker.color.a = 1.0
        self.pub5.publish(brick_marker)

    def place_func(self, request, response):
        self.brick_init_pose = request.pose
        # Brick transform
        base = TransformStamped()
        base.header.frame_id = 'world'
        base.child_frame_id = 'brick'
        time = self.get_clock().now().to_msg()
        base.header.stamp = time
        self.get_logger().info("Brick initialised") 
        self.brick.exists = True  
        self.brick.update_state(states=State.EXISTS)      
        base.transform.translation = Vector3 ( x = float(self.brick_init_pose.x), y = float(self.brick_init_pose.y), z=float(self.brick_init_pose.z)) 
        self.broadcaster.sendTransform(base)
        self.brick.transform_setter(base)
        self.brick_marker_define([self.brick_init_pose.x,self.brick_init_pose.y,self.brick_init_pose.z])        
        return response

    def drop_func(self, request, response):
        gravity = float(request.gravity.data)
        self.brick.state = State.FALLING
        # self.brick_init_pose will not exist if place is not called before drop.
        self.world = World(brick_pose=[self.brick_init_pose.x,self.brick_init_pose.y,self.brick_init_pose.z], gravity=gravity, dt = self.frequency)        
        return response

    def timer_callback(self):

        if (self.brick.state == State.EXISTS):
            time = self.get_clock().now().to_msg()
            self.brick.base.header.stamp = time
            self.broadcaster.sendTransform(self.brick.base)

        elif (self.brick.state == State.FALLING):            
            # Pose update
            self.updated_brick_pose = self.world.drop()
            self.get_logger().info(f"Started dropping at x: {self.updated_brick_pose[0]} y: {self.updated_brick_pose[1]} and z {self.updated_brick_pose[2]}")  
            if (self.updated_brick_pose[2] <= 0.6) :
               self.get_logger().info(f"Stopped dropping ")
               self.updated_brick_pose[2] = 0.0
               self.brick.state = State.DROPPED_ON_GROUND                
            else:
                pass
            # Setting the new pose
            self.world.brick = self.updated_brick_pose
            # Tranform update
            base = TransformStamped()
            base.header.frame_id = 'world'
            base.child_frame_id = 'brick'
            time = self.get_clock().now().to_msg()
            base.header.stamp = time       
            base.transform.translation = Vector3 ( x = float(self.updated_brick_pose[0]), y = float(self.updated_brick_pose[1]), z=float(self.updated_brick_pose[2])) 
            self.broadcaster.sendTransform(base)
            self.brick.transform_setter(base)        
            # Marker update
            self.brick_marker_define(self.updated_brick_pose)

        elif(self.brick.state == State.DROPPED_ON_GROUND):
            # To keep it on ground.
            # Tranform update
            self.get_logger().info(f"Stopped dropping at x: {self.updated_brick_pose[0]} y: {self.updated_brick_pose[1]} and z {self.updated_brick_pose[2]}")  
            base = TransformStamped()
            base.header.frame_id = 'world'
            base.child_frame_id = 'brick'
            time = self.get_clock().now().to_msg()
            base.header.stamp = time       
            base.transform.translation = Vector3 ( x = float(self.updated_brick_pose[0]), y = float(self.updated_brick_pose[1]), z=float(self.updated_brick_pose[2])) 
            self.broadcaster.sendTransform(base)
            self.brick.transform_setter(base)        
            # Marker update
            self.brick_marker_define(self.updated_brick_pose)



            # fall till cases !


class State(Enum):
    ABSENT = 0
    EXISTS = 1
    FALLING = 2  
    DROPPED_ON_PLATFORM = 3
    DROPPED_ON_GROUND = 4
    SLIPPING = 5
    FLOATING = 6

class Brick():
    def __init__(self):
        self.exists = False
        self.state = State.ABSENT

    def update_state(self, states):
        self.state = states
    
    def transform_setter(self, base):
        self.base = base

def main(args=None):
    rclpy.init(args=args)
    node = arena()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)