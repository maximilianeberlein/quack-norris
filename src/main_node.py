#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import os
import yaml
import message_filters

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
        image_sub = message_filters.Subscriber(f'/{self.bot_name}/camera_node/image/compressed', CompressedImage)
        calib_sub = message_filters.Subscriber(f'/{self.bot_name}/camera_node/camera_info', CameraInfo)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

        # Synchronize the image and camera info messages
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, calib_sub], queue_size=10, slop=0.1, )
        ts.registerCallback(self.image_callback)

        # Publishers
        self.pub_wheels = rospy.Publisher(f'/{self.bot_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.rect_pub = rospy.Publisher(f'/{self.bot_name}/camera_node/rect/image_rect', Image, queue_size=1)
        self.rect_info_pub = rospy.Publisher(f'/{self.bot_name}/camera_node/rect/camera_info', CameraInfo, queue_size=1)
        self.tag_ids_pub = rospy.Publisher(f'/{self.bot_name}/detected_tags', Int32MultiArray, queue_size=10)

        # Define the wheel command message
        self.wheel_cmd = WheelsCmdStamped()

        rospy.on_shutdown(self.shutdown_duckie)

        rospy.loginfo("AprilTag Detector Node initialized.")

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

    def tag_callback(self, msg):
        try:
                
            for detection in msg.detections:
                tag_id = detection.id[0]
                rospy.loginfo(f"Detected tag ID: {tag_id}")
                
                if tag_id == 20:
                    self.stop_robot()
                    return
                
                # if tag_id == 58:
                #     self.drive_backward()
                #     return    

                # if tag_id in self.apriltag_data:
                #     tag_data = self.apriltag_data[tag_id]
                #     rospy.loginfo(f"Detected tag ID: {tag_id}, Name: {tag_data[0]}, Position: {tag_data[1]}, Orientation: {tag_data[2]}")
                #     # self.tag_ids_pub.publish(Int32MultiArray(data=[tag_id]))
                #     if tag_data[0] == 'Stop':
                #         self.stop_robot()
                #     # elif tag_data[1] == 'forward':
                #     #     self.drive_forward()
                #     elif tag_data[0] == 'Backward':
                #         self.drive_backward()
                #     return
                # else:
                #     rospy.logerr(f"Tag ID {tag_id} not found in the 'apriltag_data' config file.")
        
            # If no tag with ID 20 or 58 is detected, drive forward
            #self.drive_forward()
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")
    

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
        node = MainNode()
        rospy.sleep(1)
        node.run()
    except rospy.ROSInterruptException:
        pass