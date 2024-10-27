#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from builtin_interfaces.msg import Duration
import time

class arena(Node):

    def __init__(self):
        super().__init__('arena') 
        self.frequency = 0.01

        # make it a array later.
        # Marker: Wall 1
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub1 = self.create_publisher(Marker, 'visualization_marker', markerQoS)
        self.m = Marker()
        self.m.header.frame_id = 'world'
        self.m.header.stamp = self.get_clock().now().to_msg()
        self.m.lifetime = Duration(sec=0, nanosec=0) 
        self.m.id = 1
        self.m.type = Marker.CUBE
        self.m.action = Marker.ADD
        height = 1.5
        length = 11.0
        thickness = 0.20
        self.m.scale.x = thickness
        self.m.scale.y = length
        self.m.scale.z = height
        self.m.pose.position.x = -5.5 + thickness/2
        self.m.pose.position.y = 0.0
        self.m.pose.position.z = height/2
        self.m.pose.orientation.x = 0.0
        self.m.pose.orientation.y = 0.0
        self.m.pose.orientation.z = 0.0
        self.m.pose.orientation.w = 0.0
        self.m.color.r = 0.615
        self.m.color.g = 0.20
        self.m.color.b = 1.0
        self.m.color.a = 0.70
        self.pub1.publish(self.m)

        # Marker: Wall 2
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub2 = self.create_publisher(Marker, 'visualization_marker', markerQoS)
        self.m.id = 2
        height = 1.5
        length = 11.0
        thickness = 0.20
        self.m.scale.x = thickness
        self.m.scale.y = length
        self.m.scale.z = height
        self.m.pose.position.x = 5.5 - thickness/2
        self.m.pose.position.y = 0.0
        self.m.pose.position.z = height/2
        self.pub2.publish(self.m)

        # Marker: Wall 3
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub3 = self.create_publisher(Marker, 'visualization_marker', markerQoS)
        self.m.id = 3
        height = 1.5
        length = 11.0
        thickness = 0.20
        self.m.scale.x = length
        self.m.scale.y = thickness
        self.m.scale.z = height
        self.m.pose.position.x = 0.0
        self.m.pose.position.y = 5.5 - thickness/2 
        self.m.pose.position.z = height/2
        self.pub3.publish(self.m)


        # Marker: Wall 4
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub4 = self.create_publisher(Marker, 'visualization_marker', markerQoS)
        self.m.id = 4
        height = 1.5
        length = 11.0
        thickness = 0.20
        self.m.scale.x = length
        self.m.scale.y = thickness
        self.m.scale.z = height
        self.m.pose.position.x = 0.0
        self.m.pose.position.y = -5.5 + thickness/2 
        self.m.pose.position.z = height/2
        self.pub4.publish(self.m)




def main(args=None):
    rclpy.init(args=args)
    node = arena()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)