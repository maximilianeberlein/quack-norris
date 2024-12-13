#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseStamped
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import CameraInfo
from pyquaternion import Quaternion as PyQuaternion
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from quack_norris.msg import TagInfo
import os
import yaml
import message_filters



class LocalMapNode:
    def __init__(self):
        rospy.init_node('local_map_node')
        
        self.load_parameters()
        self.bridge = CvBridge()
        self.bot_name = os.environ.get("VEHICLE_NAME")

        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.map1 = None
        self.map2 = None

        self.yaml_file = rospy.get_param('~yaml_file', '/code/catkin_ws/src/user_code/quack-norris/params/apriltags.yaml')

        self.forward_speed = self.drive_conroller_config['drive_params']['forward_speed']
        self.radius = self.drive_conroller_config['drive_params']['radius']
        self.wheel_distance = self.drive_conroller_config['drive_params']['wheel_distance']
        
        # Subscribers
        image_sub = message_filters.Subscriber(f'/{self.bot_name}/camera_node/image/compressed', CompressedImage)
        calib_sub = message_filters.Subscriber(f'/{self.bot_name}/camera_node/camera_info', CameraInfo)
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, calib_sub], queue_size=10, slop=0.1, )
        ts.registerCallback(self.image_callback)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

        # Publishers
        self.pub_wheels = rospy.Publisher(f'/{self.bot_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.rect_pub = rospy.Publisher(f'/{self.bot_name}/camera_node/rect', Image, queue_size=1)
        self.tag_ids_pub = rospy.Publisher(f'/{self.bot_name}/detected_tags', Int32MultiArray, queue_size=10)
        self.tag_info_pub = rospy.Publisher(f'/{self.bot_name}/tag_info', TagInfo, queue_size=1)
        # Define the wheel command message
        self.wheel_cmd = WheelsCmdStamped()
        self.waypoints = self.load_waypoints(self.yaml_file)



        rospy.on_shutdown(self.shutdown_duckie)

        rospy.loginfo("AprilTag Detector Node initialized.")

    def load_waypoints(self, yaml_file):
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
        return data['standalone_tags']

    def calib_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape((3, 3))
            self.dist_coeffs = np.array(msg.D)
            self.new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (msg.width, msg.height), 1, (msg.width, msg.height))
            self.map1, self.map2 = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix, (msg.width, msg.height), 5)
            rospy.loginfo("Camera calibration parameters received.")

    def image_callback(self, image_msg, calib_msg):
        
        if self.map1 is None or self.map2 is None:
            self.calib_callback(calib_msg)
        
        # Convert the image from ROS format to OpenCV format
        image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding="mono8")
        
        # Rectify the image
        rectified_image = cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)
        
        # Convert the rectified image back to ROS format
        rect_img = self.bridge.cv2_to_imgmsg(rectified_image, encoding="mono8")
        rect_img.header = image_msg.header
    
        self.rect_pub.publish(rect_img)

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
                #rospy.loginfo(f"Detected tag ID: {tag_id}, matching waypoint: {matching_waypoint['name']}")

                quaternion_q = detection.pose.pose.pose.orientation
                roll,pitch,yaw = euler_from_quaternion([quaternion_q.x,quaternion_q.y,quaternion_q.z,quaternion_q.w])
                #rospy.loginfo(f"Yaw: {np.rad2deg(yaw)}, Pitch: {np.rad2deg(pitch)}, Roll: {np.rad2deg(roll)}")
                apriltag_positioning_quat = PyQuaternion(matching_waypoint['orientation'][3], matching_waypoint['orientation'][0], matching_waypoint['orientation'][1], matching_waypoint['orientation'][2])
                reference_quat = PyQuaternion(1, 0, 0, 0)
                rotation_difference = apriltag_positioning_quat * reference_quat.inverse

                yaw = -rotation_difference.yaw_pitch_roll[0] + np.pi
                #rospy.loginfo(f"marker pos in robo frame {detection.pose.pose.pose.position.x,detection.pose.pose.pose.position.y,detection.pose.pose.pose.position.z}")
                yaw_quat = PyQuaternion(axis=[0, 0, 1], angle=-yaw)
                tag_info = TagInfo()
                tag_info.tag_id = tag_id
                tag_info.x = detection.pose.pose.pose.position.x
                tag_info.y = detection.pose.pose.pose.position.y
                tag_info.z = detection.pose.pose.pose.position.z
                tag_info.angle = np.rad2deg(pitch)
                # rospy.loginfo(f"Tag ID: {tag_id}")
                self.tag_info_pub.publish(tag_info)


    
    

    def load_parameters(self):
        # Load controller configuration
        drive_controller_file = rospy.get_param('~drive_controller_file', '/code/catkin_ws/src/user_code/quack-norris/params/drive_controller.yaml')
        self.drive_conroller_config = self.load_yaml_file(drive_controller_file)

        # apriltag_data_file = rospy.get_param('~apriltag_data_file', '/code/catkin_ws/src/user_code/quack-norris/params/apriltag_data.yaml')
        # self.apriltag_data = {}
        # for tag in self.load_yaml_file(apriltag_data_file):
        #     self.apriltag_data[tag['id']] = [tag['position'], tag['command']]

        apriltag_data_file = rospy.get_param('~apriltag_data_file', '/code/catkin_ws/src/user_code/quack-norris/params/apriltags.yaml')
        data = self.load_yaml_file(apriltag_data_file)
        self.apriltag_data = {}
        for tag in data.get('standalone_tags', []):
            self.apriltag_data[tag['id']] = [tag['name'], tag['position'], tag['orientation']]

        rospy.loginfo("Configuration parameters loaded.")

    def load_yaml_file(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

    def drive_forward(self):
        self.wheel_cmd.vel_left = self.forward_speed 
        self.wheel_cmd.vel_right = self.forward_speed
        self.pub_wheels.publish(self.wheel_cmd)
    
    def stop_robot(self):
        self.wheel_cmd.vel_left = 0
        self.wheel_cmd.vel_right = 0
        self.pub_wheels.publish(self.wheel_cmd)

    def drive_backward(self):
        self.wheel_cmd.vel_left = -self.forward_speed #* (1 - self.wheel_distance / (2 * self.radius))
        self.wheel_cmd.vel_right = -self.forward_speed  #* (1 + self.wheel_distance / (2 * self.radius))
        self.pub_wheels.publish(self.wheel_cmd)

    def shutdown_duckie(self):
        rospy.loginfo("Shutting down... stopping the robot.")
        self.pub_wheels.publish(WheelsCmdStamped())
        rospy.sleep(1)  # Allow time for the message to propagate
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LocalMapNode()
        rospy.sleep(1)
        node.run()
    except rospy.ROSInterruptException:
        pass