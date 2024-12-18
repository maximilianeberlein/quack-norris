#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import os
import yaml
import message_filters
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Point
from tf.transformations import quaternion_from_euler


class MainNode:
    def __init__(self):
        rospy.init_node('apriltag_detector_node')
        
        self.load_parameters()
        self.bridge = CvBridge()
        self.bot_name = os.environ.get("VEHICLE_NAME")

        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.map1 = None
        self.map2 = None
        
        # Subscribers
        image_sub = message_filters.Subscriber(f'/{self.bot_name}/camera_node/image/compressed', CompressedImage)
        calib_sub = message_filters.Subscriber(f'/{self.bot_name}/camera_node/camera_info', CameraInfo)
        


        # Synchronize the image and camera info messages
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, calib_sub], queue_size=10, slop=0.1, )
        ts.registerCallback(self.image_callback)

        # Publishers
        self.rect_pub = rospy.Publisher(f'/{self.bot_name}/camera_node/rect/image_rect', Image, queue_size=1)
        self.rect_info_pub = rospy.Publisher(f'/{self.bot_name}/camera_node/rect/camera_info', CameraInfo, queue_size=1)
        self.tag_ids_pub = rospy.Publisher(f'/{self.bot_name}/detected_tags', Int32MultiArray, queue_size=10)

        # Define the wheel command message
        self.wheel_cmd = WheelsCmdStamped()

        # NEW
        self.rate = rospy.Rate(10)
        
    
    def calib_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape((3, 3))
            self.dist_coeffs = np.array(msg.D)
            self.new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (msg.width, msg.height), 1, (msg.width, msg.height))
            self.map1, self.map2 = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix, (msg.width, msg.height), 5)
            rospy.loginfo("Camera calibration parameters received.")

    def image_callback(self, image_msg, camera_info_msg):
        
        if self.map1 is None or self.map2 is None:
            self.calib_callback(camera_info_msg)
        
        # Convert the image from ROS format to OpenCV format
        image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding="mono8")
        
        rectified_image = cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)

        # Convert the rectified image to a compressed image
        rectified_image_msg = self.bridge.cv2_to_imgmsg(rectified_image, encoding="mono8")

        # Publish the compressed rectified image
        rectified_image_msg.header = image_msg.header
        self.rect_pub.publish(rectified_image_msg)
        self.rect_info_pub.publish(camera_info_msg)

        # Rectify the image
        # rectified_image = cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)
        
        # # Convert the rectified image back to ROS format
        # rect_img = self.bridge.cv2_to_imgmsg(rectified_image, encoding="mono8")
        # rect_img.header = image_msg.header
    
        # self.rect_pub.publish(rect_img)    

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

        self.obstacle_data = {}

    # Extract obstacles and save to the dictionary
        for tag in data.get('obstacle_tags', []):
            self.obstacle_data[tag['id']] = [tag['name'], tag['speed']]
        

    def load_yaml_file(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
        
    

    
    
        
        # self.pub_wheels.publish(WheelsCmdStamped())
        # rospy.sleep(1)  # Allow time for the message to propagate
    
    def run(self):
        
        while not rospy.is_shutdown():
            # rospy.loginfo("Running...")
            
            rospy.spin()

if __name__ == '__main__':
    try:
        node = MainNode()
        rospy.sleep(1)
        node.run()
    except rospy.ROSInterruptException:
        pass