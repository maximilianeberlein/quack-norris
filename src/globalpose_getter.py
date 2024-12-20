#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped, Pose
from apriltag_ros.msg import AprilTagDetectionArray
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from quack_norris.msg import TagInfo
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from typing import Tuple


class LocalizationNode:
    def __init__(self):
        rospy.init_node('viz_node', anonymous=True)
        self.bot_name = os.environ.get("VEHICLE_NAME")

        # Subscribers
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.self_localization_callback)

        # Publishers
        self.globalpose_pub = rospy.Publisher('/duckiebot_globalpose', PoseWithCovarianceStamped, queue_size=1)
        self.tag_info_pub = rospy.Publisher(f'/{self.bot_name}/tag_info', TagInfo, queue_size=1)
        
        # Parameters
        self.yaml_file = rospy.get_param('~yaml_file', '/code/catkin_ws/src/user_code/quack-norris/params/apriltags.yaml')
        
        # Initialize global pose and orientation
        self.globalpose = Point(0, 0, 0.7)
        self.globalorientation = Quaternion(0, 0, 0, 1)

        # Load waypoints (i.e. apriltags) and obstacles
        self.waypoints, self.obstacles = self.load_tags(self.yaml_file)
        self.waypoint_dict = {waypoint['id']: waypoint for waypoint in self.waypoints}
        self.obstacle_dict = {obstacle['id']: obstacle for obstacle in self.obstacles}
        rospy.loginfo(f"Waypoint IDs: {list(self.waypoint_dict.keys())}")
        rospy.loginfo(f"Obstacle IDs: {list(self.obstacle_dict.keys())}")

        self.world_coords = PyQuaternion(axis=[0, 0, 1], angle=np.pi)

        self.pose_msg = PoseWithCovarianceStamped()
        self.pose_msg.header.frame_id = "map"
        self.pose_msg.pose.covariance = [1e-9 if i % 7 == 0 else 0 for i in range(36)] # Set covariance to zero for simplicity, we can adjust this as needed
        
    def load_tags(self, yaml_file: str) -> Tuple[list, list]:
        """
        Load waypoints and obstacles from a YAML file.
        """
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
        
        # Extract standalone tags
        standalone_tags = data.get('standalone_tags', [])
        
        # Separate waypoints and obstacles
        waypoints = []
        obstacles = []

        for tag in standalone_tags:
            if 'speed' in tag and 'width' in tag and 'length' in tag:  # Obstacles have speed, width, and length
                obstacles.append(tag)
            else: # Tags without these properties are waypoints
                waypoints.append(tag)

        rospy.loginfo(f"Loaded {len(waypoints)} waypoints and {len(obstacles)} obstacles.")
        return waypoints, obstacles

    def self_localization_callback(self, msg) -> None:
        """
        Callback function for the AprilTag detection subscriber.
        Publishes the global pose of the Duckiebot.
        """
        closest_detection = None
        min_distance = float('inf')

        # Find closest apriltag and for use in global localization
        for detection in msg.detections:
            position = detection.pose.pose.pose.position
            position_array = np.array([position.x, position.y, position.z])
            distance = np.linalg.norm(position_array)
            if distance < min_distance:
                min_distance = distance
                closest_detection = detection

        if closest_detection:       # If an apriltag is detected
            if min_distance < 0.9:  # Ignore apriltags that are too far away, as they are inaccurate
                angle = np.arctan2(closest_detection.pose.pose.pose.position.z, closest_detection.pose.pose.pose.position.x)
                if np.radians(45) < angle < np.radians(135): # Ignore apriltags that are at an extreme angle, as they are inaccurate
                    tag_id = closest_detection.id[0]

                    # Use apriltag waypoints instead of detection (internal usage)
                    if tag_id in self.waypoint_dict:
                        matching_waypoint = self.waypoint_dict[tag_id]
                        matching_waypoint = self.waypoint_dict.get(tag_id, None)

                        if matching_waypoint:
                            # Publish tag info
                            tag_info = TagInfo()
                            tag_info.tag_id = tag_id
                            tag_info.y = -closest_detection.pose.pose.pose.position.x
                            tag_info.z = closest_detection.pose.pose.pose.position.y
                            tag_info.x = closest_detection.pose.pose.pose.position.z
                            tag_info.angle = np.rad2deg(0)
                            self.tag_info_pub.publish(tag_info)

                            ########### Some calculations later used for the global pose of the Duckie ###########
                            camera_to_apriltag_transform = PyQuaternion(
                                                            closest_detection.pose.pose.pose.orientation.w,
                                                            closest_detection.pose.pose.pose.orientation.x,
                                                            closest_detection.pose.pose.pose.orientation.y,
                                                            closest_detection.pose.pose.pose.orientation.z)
                            waypoint_orientation = PyQuaternion(
                                                            matching_waypoint['orientation'][3],
                                                            matching_waypoint['orientation'][0],
                                                            matching_waypoint['orientation'][1],
                                                            matching_waypoint['orientation'][2])
                            yaw_sim = (waypoint_orientation * self.world_coords.inverse).yaw_pitch_roll[2]
                            yaw_real = euler_from_quaternion([camera_to_apriltag_transform.x, camera_to_apriltag_transform.y, camera_to_apriltag_transform.z, camera_to_apriltag_transform.w])[1]
                            total_yaw = yaw_sim - yaw_real
                            rotate = total_yaw + np.pi
                            combined_rotation_matrix = np.array([
                                [np.cos(rotate), -np.sin(rotate)],
                                [np.sin(rotate), np.cos(rotate)]
                            ])
                            relative_duckie_pos_sim_transformed = np.dot(combined_rotation_matrix,
                                                                        np.array([-closest_detection.pose.pose.pose.position.z, -closest_detection.pose.pose.pose.position.x]))
                            self.globalpose = Point(
                                matching_waypoint['position'][0] - relative_duckie_pos_sim_transformed[0],
                                matching_waypoint['position'][1] + relative_duckie_pos_sim_transformed[1],
                                0.07)
                            self.globalorientation = quaternion_from_euler(0, 0, np.arctan2(np.sin(total_yaw), np.cos(total_yaw))) 

                            # Publish global pose
                            self.pose_msg.header.stamp = rospy.Time.now()
                            self.pose_msg.pose.pose = Pose(
                                position=self.globalpose,
                                orientation=Quaternion(-self.globalorientation[0],
                                                       -self.globalorientation[1],
                                                       -self.globalorientation[2],
                                                       self.globalorientation[3]))
                            self.globalpose_pub.publish(self.pose_msg)

                    elif tag_id in self.obstacle_dict:
                        # Publish tag info
                        tag_info = TagInfo()
                        tag_info.tag_id = tag_id
                        tag_info.y = -closest_detection.pose.pose.pose.position.x
                        tag_info.z = closest_detection.pose.pose.pose.position.y
                        tag_info.x = closest_detection.pose.pose.pose.position.z
                        tag_info.angle = np.rad2deg(0)
                        self.tag_info_pub.publish(tag_info)


if __name__ == "__main__":
    try:
        node = LocalizationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass