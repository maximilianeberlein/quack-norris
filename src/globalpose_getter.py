#!/usr/bin/env python3

import rospy
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseWithCovarianceStamped, Pose
from apriltag_ros.msg import AprilTagDetectionArray
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from nav_msgs.msg import Odometry
from quack_norris.msg import TagInfo
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class LocalizationNode:
    def __init__(self):
        rospy.init_node('viz_node', anonymous=True)
        
        self.bot_name = os.environ.get("VEHICLE_NAME")

        self.globalpose_pub = rospy.Publisher('/duckiebot_globalpose', PoseWithCovarianceStamped, queue_size=1)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.self_localization_callback)    
        self.tag_info_pub = rospy.Publisher(f'/{self.bot_name}/tag_info', TagInfo, queue_size=1)

        self.yaml_file = rospy.get_param('~yaml_file', '/code/catkin_ws/src/user_code/quack-norris/params/apriltags.yaml')
        
        self.globalpose = Point(0, 0, 0.7)
        self.globalorientation = Quaternion(0, 0, 0, 1)

        self.waypoints = self.load_waypoints(self.yaml_file)
        self.waypoint_dict = {waypoint['id']: waypoint for waypoint in self.waypoints}
        self.world_coords = PyQuaternion(axis=[0, 0, 1], angle=np.pi)

        self.pose_msg = PoseWithCovarianceStamped()
        self.pose_msg.header.frame_id = "map"
        # Set covariance to zero for simplicity, we can adjust this as needed
        self.pose_msg.pose.covariance = [1e-9 if i % 7 == 0 else 0 for i in range(36)]
        

    def load_waypoints(self, yaml_file):
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
        return data['standalone_tags']


    def self_localization_callback(self, msg):
        closest_detection = None
        min_distance = float('inf')

        # find closest apriltag and use it for global localization
        for detection in msg.detections:
            position = detection.pose.pose.pose.position
            position_array = np.array([position.x, position.y, position.z])
            distance = np.linalg.norm(position_array)
            
            if distance < min_distance:
                min_distance = distance
                closest_detection = detection

        if closest_detection:

            if min_distance < 0.6:
                angle = np.arctan2(closest_detection.pose.pose.pose.position.z, closest_detection.pose.pose.pose.position.x)
                if np.radians(45) < angle < np.radians(135):

                    # get id of apriltag and compare it with global waypoint ids
                    tag_id = closest_detection.id[0]
                    matching_waypoint = self.waypoint_dict.get(tag_id, None)

                    # this part needs some comments, some stuff was only found by trial and error
                    if matching_waypoint:
                        tag_info = TagInfo()
                        tag_info.tag_id = tag_id
                        tag_info.y = -closest_detection.pose.pose.pose.position.x
                        tag_info.z = closest_detection.pose.pose.pose.position.y
                        tag_info.x = closest_detection.pose.pose.pose.position.z
                        tag_info.angle = np.rad2deg(0)
                        # rospy.loginfo(f"Tag ID: {tag_id}, {tag_info.x,tag_info.y,tag_info.z}")
                        self.tag_info_pub.publish(tag_info)
                        camera_to_apriltag_transform = PyQuaternion(closest_detection.pose.pose.pose.orientation.w, closest_detection.pose.pose.pose.orientation.x, closest_detection.pose.pose.pose.orientation.y, closest_detection.pose.pose.pose.orientation.z)
                        waypoint_orientation = PyQuaternion(matching_waypoint['orientation'][3], matching_waypoint['orientation'][0], matching_waypoint['orientation'][1], matching_waypoint['orientation'][2])
                        yaw_sim = (waypoint_orientation * self.world_coords.inverse).yaw_pitch_roll[2]
                        yaw_real = euler_from_quaternion([camera_to_apriltag_transform.x, camera_to_apriltag_transform.y, camera_to_apriltag_transform.z, camera_to_apriltag_transform.w])[1]

                        total_yaw = yaw_sim - yaw_real

                        rotate = total_yaw + np.pi
                        combined_rotation_matrix = np.array([
                            [np.cos(rotate), -np.sin(rotate)],
                            [np.sin(rotate), np.cos(rotate)]
                        ])

                        relative_duckie_pos_sim_transformed = np.dot(combined_rotation_matrix, np.array([-closest_detection.pose.pose.pose.position.z, -closest_detection.pose.pose.pose.position.x]))
                          
                        self.globalpose = Point(
                            matching_waypoint['position'][0] - relative_duckie_pos_sim_transformed[0],
                            matching_waypoint['position'][1] + relative_duckie_pos_sim_transformed[1],
                            0.07
                        )
                        self.globalorientation = quaternion_from_euler(0, 0, np.arctan2(np.sin(total_yaw), np.cos(total_yaw))) 

                        self.pose_msg.header.stamp = rospy.Time.now()
                        self.pose_msg.pose.pose = Pose(
                            position=self.globalpose,
                            orientation= Quaternion(-self.globalorientation[0], -self.globalorientation[1], -self.globalorientation[2], self.globalorientation[3])
                        )
                        self.globalpose_pub.publish(self.pose_msg)

if __name__ == "__main__":
    try:
        node = LocalizationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass