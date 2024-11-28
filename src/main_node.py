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


from cv_bridge import CvBridge
import os
import yaml

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

        self.forward_speed = self.drive_conroller_config['drive_params']['forward_speed']
        self.radius = self.drive_conroller_config['drive_params']['radius']
        self.wheel_distance = self.drive_conroller_config['drive_params']['wheel_distance']
        
        # Subscribers
        self.image_sub = rospy.Subscriber(f'/{self.bot_name}/camera_node/image/compressed', CompressedImage, self.image_callback)
        self.calibration_sub = rospy.Subscriber(f'/{self.bot_name}/camera_node/camera_info', CameraInfo, self.calib_callback)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

        # Publishers
        self.pub_wheels = rospy.Publisher(f'/{self.bot_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.rect_pub = rospy.Publisher(f'/{self.bot_name}/camera_node/rect', Image, queue_size=1)
        self.tag_ids_pub = rospy.Publisher(f'/{self.bot_name}/detected_tags', Int32MultiArray, queue_size=10)

        # Define the wheel command message
        self.wheel_cmd = WheelsCmdStamped()

        rospy.loginfo("AprilTag Detector Node initialized.")

    def calib_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape((3, 3))
            self.dist_coeffs = np.array(msg.D)
            self.new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (msg.width, msg.height), 1, (msg.width, msg.height))
            self.map1, self.map2 = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix, (msg.width, msg.height), 5)
            rospy.loginfo("Camera calibration parameters received.")

    def image_callback(self, msg):
        
        if self.map1 is None or self.map2 is None:
            rospy.logwarn("Camera calibration parameters not yet received.")
            return
        
        # Convert the image from ROS format to OpenCV format
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="mono8")
        
        # Rectify the image
        rectified_image = cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)
        
        # Convert the rectified image back to ROS format
        rect_img = self.bridge.cv2_to_imgmsg(rectified_image, encoding="mono8")
        rect_img.header = msg.header
        self.rect_pub.publish(rect_img)

    def tag_callback(self, msg):
        try:
                
            for detection in msg.detections:
                tag_id = detection.id[0]
               # rospy.loginfo(f"Detected tag ID: {tag_id}")
                
                if tag_id == 20:
                    self.stop_robot()
                    return
                
                if tag_id == 58:
                    self.drive_backward()
                    return    
        
            # If no tag with ID 20 or 58 is detected, drive forward
            self.drive_forward()
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")
    

    def load_parameters(self):
        # Load controller configuration
        drive_controller_file = rospy.get_param('~drive_controller_file', '/code/catkin_ws/src/user_code/quack-norris/params/drive_controller.yaml')
        self.drive_conroller_config = self.load_yaml_file(drive_controller_file)

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
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = MainNode()
        rospy.sleep(1)
        node.run()
    except rospy.ROSInterruptException:
        pass