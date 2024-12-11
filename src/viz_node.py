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
        
        self.marker_pub = rospy.Publisher('global_map_viz', MarkerArray, queue_size=10)
        self.duckiebot_pub = rospy.Publisher('duckiebot_viz', Marker, queue_size=10)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.self_localization_callback)
        
        self.yaml_file = rospy.get_param('~yaml_file', '/code/catkin_ws/src/user_code/quack-norris/params/apriltags.yaml')
        self.maps_yaml_file = rospy.get_param('~maps_yaml_file', '/code/catkin_ws/src/user_code/quack-norris/params/maps.yaml')
        self.map_name = rospy.get_param('~map_name', 'lap_big')
        
        rospy.loginfo(f"Selected map: {self.map_name}")
        
        self.maps = self.load_maps(self.maps_yaml_file)
        self.map_params = self.get_map_params(self.maps, self.map_name)
        self.qx = PyQuaternion(np.sqrt(0.5), np.sqrt(0.5), 0, 0) # 90 degree rot around x-axis
        self.camera_angle = PyQuaternion(0.5, 0.8660254, 0.0, 0.0) # rot around x-axis corresponding to camera angle on duckiebot
        self.qz = PyQuaternion(np.sqrt(0.5), 0, 0, np.sqrt(0.5)) # 90 degree rot around z-axis
        
        if not self.map_params:
            rospy.logerr(f"Map '{self.map_name}' not found in {self.maps_yaml_file}")
            return
                
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
            marker.pose.orientation.w = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 1.0
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
            text_marker.scale.z = 0.1 
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.text = f"{waypoint['name']} ({waypoint['id']})"

            self.marker_array.markers.append(text_marker)

            # Create the arrow marker for waypoint orientation indication
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
            arrow_marker.pose.orientation.w = waypoint['orientation'][0]
            arrow_marker.pose.orientation.x = waypoint['orientation'][1]
            arrow_marker.pose.orientation.y = waypoint['orientation'][2]
            arrow_marker.pose.orientation.z = waypoint['orientation'][3]
            arrow_marker.scale.x = 0.2  # Arrow length
            arrow_marker.scale.y = 0.05  # Arrow width
            arrow_marker.scale.z = 0.05  # Arrow height
            arrow_marker.color.a = 1.0
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 1.0
            arrow_marker.color.b = 0.0

            self.marker_array.markers.append(arrow_marker)

    def self_localization_callback(self, msg):
        closest_detection = None
        min_distance = float('inf')

        # find closest apriltag and use it for global localization
        for detection in msg.detections:
            position = detection.pose.pose.pose.position
            distance = np.sqrt(position.x**2 + position.y**2 + position.z**2)
            
            if distance < min_distance:
                min_distance = distance
                closest_detection = detection


        if closest_detection:

            # get id of apriltag and compare it with global waypoint ids
            tag_id = closest_detection.id[0]
            
            matching_waypoint = None
            for waypoint in self.waypoints:
                if waypoint['id'] == tag_id:
                    matching_waypoint = waypoint
                    break

            # this part needs some comments, some stuff was only found by trial and error
            if matching_waypoint:

                camera_to_apriltag_transform = PyQuaternion(closest_detection.pose.pose.pose.orientation.w, closest_detection.pose.pose.pose.orientation.x, closest_detection.pose.pose.pose.orientation.y, closest_detection.pose.pose.pose.orientation.z)

                # transform apriltag position with respect to real duckie to sim-duckie pos with respect to waypoint
                relative_duckie_pos_sim = Point(
                    -closest_detection.pose.pose.pose.position.z,
                    closest_detection.pose.pose.pose.position.x,
                    0.0335
                )

                # get z axis rotation between waypoint and global coordinate system
                waypoint_orientation = PyQuaternion(matching_waypoint['orientation'][3], matching_waypoint['orientation'][0], matching_waypoint['orientation'][1], matching_waypoint['orientation'][2])
                global_map_orientation = PyQuaternion(1, 0, 0, 0)
                yaw = (waypoint_orientation * global_map_orientation.inverse).yaw_pitch_roll[2]
                yaw_quat = PyQuaternion(axis=[0, 0, 1], angle=yaw)


                # rotate relative pos of sim-duckie to waypoint by its yaw in reference to the global coords.
                rotated_rel_duckie_pos_sim_x = relative_duckie_pos_sim.x * np.cos(-yaw) - relative_duckie_pos_sim.y * np.sin(-yaw)
                rotated_rel_duckie_pos_sim_y = relative_duckie_pos_sim.x * np.sin(-yaw) + relative_duckie_pos_sim.y * np.cos(-yaw)

                # set global position of sim-duckie
                self.duckiebot_marker.pose.position = Point(
                    matching_waypoint['position'][0] + rotated_rel_duckie_pos_sim_x,
                    matching_waypoint['position'][1] + rotated_rel_duckie_pos_sim_y,
                    0.0335
                )

                # set orientation of sim-duckie (qz and qy rotations due to different reference coord systems of duckie in sim vs real)
                orientation = self.qz.inverse * self.qx.inverse * camera_to_apriltag_transform * self.camera_angle.inverse * yaw_quat * self.qz 

                # set global orientation of sim-duckie
                self.duckiebot_marker.pose.orientation = Quaternion(-orientation[1], -orientation[2], -orientation[3], orientation[0])


    def run(self):
        rate = rospy.Rate(10) 

        while not rospy.is_shutdown():
            
            self.marker_pub.publish(self.marker_array)
            self.duckiebot_pub.publish(self.duckiebot_marker)
            rate.sleep()

if __name__ == "__main__":
    try:
        node = VizNode()
        node.run()
    except rospy.ROSInterruptException:
        pass