#!/usr/bin/env python3

import rospy
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3
from apriltag_ros.msg import AprilTagDetectionArray
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
class VizNode:
    def __init__(self):
        rospy.init_node('viz_node', anonymous=True)
        
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.duckiebot_pub = rospy.Publisher('duckiebot_marker', Marker, queue_size=10)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        
        self.yaml_file = rospy.get_param('~yaml_file', '/code/catkin_ws/src/user_code/quack-norris/params/apriltags.yaml')
        self.maps_yaml_file = rospy.get_param('~maps_yaml_file', '/code/catkin_ws/src/user_code/quack-norris/params/maps.yaml')
        self.map_name = rospy.get_param('~map_name', 'lap_big')
        
        rospy.loginfo(f"Selected map: {self.map_name}")
        
        self.maps = self.load_maps(self.maps_yaml_file)
        self.map_params = self.get_map_params(self.maps, self.map_name)
        self.qx = PyQuaternion(np.sqrt(0.5), np.sqrt(0.5), 0, 0)
        self.camera_angle = PyQuaternion(0.5, 0.8660254, 0.0, 0.0)
        self.qz = PyQuaternion(np.sqrt(0.5), 0, 0, np.sqrt(0.5))
        
        if not self.map_params:
            rospy.logerr(f"Map '{self.map_name}' not found in {self.maps_yaml_file}")
            return
        
        rospy.loginfo(f"Map parameters: {self.map_params}")
        
        self.waypoints = self.load_waypoints(self.yaml_file)
        self.marker_array = MarkerArray()
        self.create_markers()
        
        self.duckiebot_marker = Marker()
        self.duckiebot_marker.header.frame_id = "map"
        self.duckiebot_marker.header.stamp = rospy.Time.now()
        self.duckiebot_marker.ns = "duckiebot"
        self.duckiebot_marker.id = 0
        self.duckiebot_marker.type = Marker.MESH_RESOURCE
        self.duckiebot_marker.action = Marker.ADD
        self.duckiebot_marker.mesh_resource = "file:///code/catkin_ws/src/user_code/quack-norris/map_files/duckiebot-blue.dae"
        self.duckiebot_marker.mesh_use_embedded_materials = True
        z_offset_mesh = 0.0335  # offset such that the wheels are on the ground
        self.duckiebot_marker.pose.position = Point(0, 0, z_offset_mesh)
        self.duckiebot_marker.pose.orientation = Quaternion(0, 0, 0, 1)
        self.duckiebot_marker.scale = Vector3(1, 1, 1)
        

    def load_maps(self, yaml_file):
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
        return data['maps']

    def get_map_params(self, maps, map_name):
        for map_entry in maps:
            if map_entry['name'] == map_name:
                return map_entry
        return None

    def load_waypoints(self, yaml_file):
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
        return data['standalone_tags']

    def create_markers(self):
        plane_width = self.map_params['plane_width']
        mesh_path = self.map_params['mesh_path']
        
        # Create the marker for the map
        map_marker = Marker()
        map_marker.header.frame_id = "map"
        map_marker.header.stamp = rospy.Time.now()
        map_marker.ns = "map"
        map_marker.id = 0
        map_marker.type = Marker.MESH_RESOURCE
        map_marker.action = Marker.ADD
        map_marker.pose.position.x = -plane_width / 2.0
        map_marker.pose.position.y = -plane_width / 2.0
        map_marker.pose.position.z = 0.0
        map_marker.pose.orientation.x = 0.0
        map_marker.pose.orientation.y = 0.0
        map_marker.pose.orientation.z = 0.0
        map_marker.pose.orientation.w = 1.0
        map_marker.scale.x = 1.0
        map_marker.scale.y = 1.0
        map_marker.scale.z = 1.0
        map_marker.color.a = 1.0
        map_marker.color.r = 1.0
        map_marker.color.g = 1.0
        map_marker.color.b = 1.0
        map_marker.mesh_resource = mesh_path
        map_marker.mesh_use_embedded_materials = True

        self.marker_array.markers.append(map_marker)
        
        for waypoint in self.waypoints:
            # Create the marker for the waypoint
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = waypoint['id']
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint['position'][0]
            marker.pose.position.y = waypoint['position'][1]
            marker.pose.position.z = 0.0  # Z position is zero for all waypoints
            marker.pose.orientation.x = waypoint['orientation'][0]
            marker.pose.orientation.y = waypoint['orientation'][1]
            marker.pose.orientation.z = waypoint['orientation'][2]
            marker.pose.orientation.w = waypoint['orientation'][3]
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            self.marker_array.markers.append(marker)

            # Create the text marker for the waypoint name
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "waypoints"
            text_marker.id = waypoint['id'] + 1000  # Ensure unique ID for text marker
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = waypoint['position'][0]
            text_marker.pose.position.y = waypoint['position'][1]
            text_marker.pose.position.z = 0.2  # Slightly above the waypoint marker
            text_marker.pose.orientation.x = 0.0
            text_marker.pose.orientation.y = 0.0
            text_marker.pose.orientation.z = 0.0
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.1  # Text height
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.text = f"{waypoint['name']} ({waypoint['id']})"

            self.marker_array.markers.append(text_marker)

            # Create the arrow marker for the waypoint orientation
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "map"
            arrow_marker.header.stamp = rospy.Time.now()
            arrow_marker.ns = "waypoints"
            arrow_marker.id = waypoint['id'] + 2000  # Ensure unique ID for arrow marker
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = waypoint['position'][0]
            arrow_marker.pose.position.y = waypoint['position'][1]
            arrow_marker.pose.position.z = 0.0  # Z position is zero for all waypoints
            arrow_marker.pose.orientation.x = waypoint['orientation'][0]
            arrow_marker.pose.orientation.y = waypoint['orientation'][1]
            arrow_marker.pose.orientation.z = waypoint['orientation'][2]
            arrow_marker.pose.orientation.w = waypoint['orientation'][3]
            arrow_marker.scale.x = 0.2  # Arrow length
            arrow_marker.scale.y = 0.05  # Arrow width
            arrow_marker.scale.z = 0.05  # Arrow height
            arrow_marker.color.a = 1.0
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 1.0
            arrow_marker.color.b = 0.0

            self.marker_array.markers.append(arrow_marker)

    def tag_callback(self, msg):

        # Process the AprilTag detections
        for detection in msg.detections:

            apriltag_transform = PyQuaternion(detection.pose.pose.pose.orientation.w, detection.pose.pose.pose.orientation.x, detection.pose.pose.pose.orientation.y, detection.pose.pose.pose.orientation.z)
            
            tag_id = detection.id[0]
            matching_waypoint = None
            for waypoint in self.waypoints:
                if waypoint['id'] == tag_id:
                    matching_waypoint = waypoint
                    break

            # this part needs some comments, some stuff was only found by trial and error
            if matching_waypoint:
                rospy.loginfo(f"Detected tag ID: {tag_id}, matching waypoint: {matching_waypoint['name']}")

                translated_pos = Point(
                    -detection.pose.pose.pose.position.z,
                    detection.pose.pose.pose.position.x,
                    0.0335
                )

                apriltag_positioning_quat = PyQuaternion(matching_waypoint['orientation'][3], matching_waypoint['orientation'][0], matching_waypoint['orientation'][1], matching_waypoint['orientation'][2])
                reference_quat = PyQuaternion(1, 0, 0, 0)
                rotation_difference = apriltag_positioning_quat * reference_quat.inverse

                yaw = -rotation_difference.yaw_pitch_roll[0] + np.pi
                # rospy.loginfo(f"Yaw (rotation around z-axis): {yaw}")
                yaw_quat = PyQuaternion(axis=[0, 0, 1], angle=-yaw)

                x_new = translated_pos.x * np.cos(yaw) - translated_pos.y * np.sin(yaw)
                y_new = translated_pos.x * np.sin(yaw) + translated_pos.y * np.cos(yaw)

                self.duckiebot_marker.pose.position = Point(
                    matching_waypoint['position'][0] + x_new,
                    matching_waypoint['position'][1] + y_new,
                    0.0335
                )

                orientation = self.qz.inverse * self.qx.inverse * apriltag_transform * self.camera_angle.inverse * yaw_quat * self.qz 
                self.duckiebot_marker.pose.orientation = Quaternion(-orientation[1], -orientation[2], -orientation[3], orientation[0])

    def run(self):
        marker_rate = rospy.Rate(1)  # 1 Hz for markers
        duckiebot_rate = rospy.Rate(10)  # 10 Hz for duckiebot

        while not rospy.is_shutdown():
            # Publish the marker array at a lower rate
            if marker_rate.remaining() == 0:
                self.marker_pub.publish(self.marker_array)
                marker_rate.sleep()

            # Publish the duckiebot marker at a higher rate
            self.duckiebot_pub.publish(self.duckiebot_marker)
            duckiebot_rate.sleep()

if __name__ == "__main__":
    try:
        node = VizNode()
        node.run()
    except rospy.ROSInterruptException:
        pass