#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import copy

class RmfMarkerTransformer(Node):
    def __init__(self):
        super().__init__('rmf_marker_transformer')
        
        # Dynamic parameters
        self.declare_parameter('fleet_marker_scale', 0.5)
        
        # Transformation offsets (RMF to ROS/Gazebo):
        # x_ros = x_rmf - 10.0
        # y_ros = y_rmf + 9.0 (Map-based)
        # y_ros = -10.0 - y_rmf (Fleet/Schedule-based)
        self.x_offset = -10.0
        self.y_offset = -10.0
        self.y_offset_map = 9.0
        
        self.subs = []
        self.pubs = []
        
        # (input_topic, output_topic)
        topics = [
            ('/map_markers', '/map_markers_ros'),
            ('/schedule_markers', '/schedule_markers_ros'),
            ('/fleet_markers', '/fleet_markers_ros'),
            ('/building_systems_markers', '/building_systems_markers_ros')
        ]
        
        for input_topic, output_topic in topics:
            is_map_based = (input_topic in ['/map_markers', '/building_systems_markers'])
            
            # Setup QoS Profile matching the publisher QoS
            qos_profile = rclpy.qos.QoSProfile(depth=10)
            if is_map_based:
                qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
                qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
            else:
                qos_profile.durability = rclpy.qos.DurabilityPolicy.VOLATILE
                qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
                
            pub = self.create_publisher(MarkerArray, output_topic, qos_profile)
            self.pubs.append(pub)
            
            # Create the callback for subscription
            sub = self.create_subscription(
                MarkerArray,
                input_topic,
                self.make_callback(pub, is_map_based, input_topic),
                qos_profile
            )
            self.subs.append(sub)
            self.get_logger().info(f"Transforming: {input_topic} -> {output_topic} (is_map_based={is_map_based})")
 
    def make_callback(self, pub, is_map_based, topic_name):
        def callback(msg):
            transformed_msg = MarkerArray()
            fleet_scale = self.get_parameter('fleet_marker_scale').value
            
            for marker in msg.markers:
                # Deep copy to avoid mutating the incoming message in place
                new_marker = copy.deepcopy(marker)
                
                # Set fixed frame to map to align with Nav2
                new_marker.header.frame_id = 'map'
                
                # Sanitize quaternion: if uninitialized (0, 0, 0, 0), set to identity (0, 0, 0, 1)
                orientation = new_marker.pose.orientation
                norm_sq = (orientation.x**2 + 
                           orientation.y**2 + 
                           orientation.z**2 + 
                           orientation.w**2)
                if norm_sq < 1e-5:
                    orientation.x = 0.0
                    orientation.y = 0.0
                    orientation.z = 0.0
                    orientation.w = 1.0
                
                # Transform pose position only if the marker has no points
                if not new_marker.points:
                    new_marker.pose.position.x = new_marker.pose.position.x + self.x_offset
                    if is_map_based:
                        # Map based markers (from building_map_server/navgraph_visualizer):
                        # y_ros = y_rmf + 9.0
                        new_marker.pose.position.y = new_marker.pose.position.y + self.y_offset_map
                    else:
                        # Fleet and Schedule markers (from fleet adapter/schedule):
                        # y_ros = -10.0 - y_rmf
                        new_marker.pose.position.y = self.y_offset - new_marker.pose.position.y
                        # Reflect Y axis means negating yaw, which negates the z component in quaternion
                        new_marker.pose.orientation.z = -new_marker.pose.orientation.z
                
                # Transform individual points (used for line strips like lanes/schedules/waypoints)
                for pt in new_marker.points:
                    pt.x = pt.x + self.x_offset
                    if is_map_based:
                        pt.y = pt.y + self.y_offset_map
                    else:
                        pt.y = self.y_offset - pt.y
                    
                # Apply dynamic scaling to fleet markers (robots and labels)
                if topic_name == '/fleet_markers':
                    new_marker.scale.x *= fleet_scale
                    new_marker.scale.y *= fleet_scale
                    new_marker.scale.z *= fleet_scale
                    
                transformed_msg.markers.append(new_marker)
            pub.publish(transformed_msg)
        return callback

def main(args=None):
    rclpy.init(args=args)
    node = RmfMarkerTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
