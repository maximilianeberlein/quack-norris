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

TILE_SIZE = 0.585

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

        self.forward_speed = self.drive_conroller_config['drive_params']['max_speed']
        # self.radius = self.drive_conroller_config['drive_params']['radius']
        self.wheel_distance = self.drive_conroller_config['drive_params']['wheel_distance']
        
        # Subscribers
        image_sub = message_filters.Subscriber(f'/{self.bot_name}/camera_node/image/compressed', CompressedImage)
        calib_sub = message_filters.Subscriber(f'/{self.bot_name}/camera_node/camera_info', CameraInfo)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        self.odom_sub = rospy.Subscriber('/wheel_encoder/odom', Odometry, self.odom_callback)


        # Synchronize the image and camera info messages
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, calib_sub], queue_size=10, slop=0.1, )
        ts.registerCallback(self.image_callback)

        # Publishers
        self.pub_wheels = rospy.Publisher(f'/{self.bot_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.rect_pub = rospy.Publisher(f'/{self.bot_name}/camera_node/rect/image_rect', Image, queue_size=1)
        self.rect_info_pub = rospy.Publisher(f'/{self.bot_name}/camera_node/rect/camera_info', CameraInfo, queue_size=1)
        self.tag_ids_pub = rospy.Publisher(f'/{self.bot_name}/detected_tags', Int32MultiArray, queue_size=10)
        self.line_follow_pub = rospy.Publisher(f"/{self.bot_name}/joy_mapper_node/joystick_override", BoolStamped, queue_size=1)

        # Define the wheel command message
        self.wheel_cmd = WheelsCmdStamped()

        # NEW
        self.rate = rospy.Rate(10)
        self.tag_detected = False
        EMERGENY_LEVEL = 1 # In [0.6, 1] - 0 is no emergency (no speed), 1 is highest emergency (max speed)
        self.desired_speed = EMERGENY_LEVEL * self.forward_speed
        self.desired_radius = self.speed_to_radius(self.desired_speed) # In range [0.585, 0.8775], depening on the urgency
        self.desired_distance = self.desired_radius + TILE_SIZE / 4
        self.desired_direction = -1 # -1 for left, 1 for right # TODO: Make this dynamic

        self.posx = 0
        self.posy = 0
        self.theta = 0

        rospy.on_shutdown(self.shutdown_duckie)

        rospy.loginfo("AprilTag Detector Node initialized.")

    def speed_to_radius(self, speed):
        return (5 * TILE_SIZE * speed) /(4 * self.forward_speed)

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
            closest_detection = None
            min_distance = float('inf')
            for detection in msg.detections:
                position = detection.pose.pose.pose.position
                position_array = np.array([position.x, position.y, position.z])
                distance = np.linalg.norm(position_array)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_detection = detection

            if closest_detection:
                # TODO: Just choose closest tag or similar
                # rospy.loginfo(f"Detected tag with ID: {detection.id[0]}")
                distance = closest_detection.pose.pose.pose.position.z # TODO: Make better
                if not distance < self.desired_distance:
                    # rospy.loginfo(f"Distance to tag: {distance}")
                    return
                # Out of range
                # if distance > 0.8775: 
                #     return
                rospy.loginfo(f"Close enough to tag: {distance}")
                
                
                # # TODO: Instead of if-statements, make curve radius depending on the urgency (resp. speed)
                # if distance <= 0.585: # Small curve range
                #     self.desired_radius = distance
                # elif distance <= 0.73125: # Medium curve range
                #     pass
                # else: # Big curve range
                #     pass
                self.tag_detected = True
                    
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
        
    def odom_callback(self, msg):
        self.posx = msg.pose.pose.position.x
        self.posy = msg.pose.pose.position.y
        # Orientation handling is just an approximation:
        # Assuming small rotations around z. Adjust if you have quaternions:
        # Convert quaternion to yaw if needed:
        q = msg.pose.pose.orientation
        # Simple approximation since you're only using z (Not accurate for all orientations)
        # Better: use tf.transformations.euler_from_quaternion
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        # self.theta = self.normalize_angle(yaw)
        self.theta = yaw

    def normalize_angle(self, angle):
        # Normalize angle to [-pi, pi]
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def drive_forward(self):
        self.wheel_cmd.vel_left = self.desired_speed 
        self.wheel_cmd.vel_right = self.desired_speed
        self.pub_wheels.publish(self.wheel_cmd)

    def drive_curve(self):
        self.line_follow_pub.publish(BoolStamped(data=True))
        self.stop_robot(2)

        init_theta = self.theta
        rospy.logwarn(f"Init theta: {init_theta}")

        rospy.logwarn("Driving curve")
        radius = self.desired_radius / 2.2 # Needed bc of duckiebot bad wheels
        # rospy.loginfo(f"Radius: {radius}")
        gain_wheeldiff = 2.0
        self.wheel_cmd.vel_left = self.desired_speed * (1 + gain_wheeldiff * np.sign(self.desired_direction) * self.wheel_distance / (2 * radius))
        self.wheel_cmd.vel_right = self.desired_speed * (1 - gain_wheeldiff * np.sign(self.desired_direction) * self.wheel_distance / (2 * radius))
        # rospy.loginfo(f"Left: {self.wheel_cmd.vel_left}, Right: {self.wheel_cmd.vel_right}")
        self.pub_wheels.publish(self.wheel_cmd)
        # duration = (np.pi * self.desired_radius)/(self.desired_speed)
        # rospy.sleep(duration)
        # while self.normalize_angle(self.theta) < self.normalize_angle(init_theta + np.pi/2): # TODO: Curve type
        if -np.pi < self.normalize_angle(init_theta + np.pi/2) < -np.pi/2:
            check_theta = self.theta - 2*np.pi
        else:
            check_theta = self.theta
        while check_theta < self.normalize_angle(init_theta + np.pi/2): # TODO: Curve type
            rospy.logwarn(f"check_theta: {check_theta}, self.theta: {check_theta}. Desired: {self.normalize_angle(init_theta + np.pi/2)}")

            if -np.pi < self.normalize_angle(init_theta + np.pi/2) < -np.pi/2:
                check_theta = self.theta - 2*np.pi
            else:
                check_theta = self.theta
            rospy.sleep(0.1)
        rospy.logwarn("Stopping curve")
        self.tag_detected = False

        self.stop_robot()
        self.line_follow_pub.publish(BoolStamped(data=False))
    
    def stop_robot(self, stop_time: float = 1):
        # self.wheel_cmd.vel_left = 0
        # self.wheel_cmd.vel_right = 0
        # self.pub_wheels.publish(self.wheel_cmd)
        self.pub_wheels.publish(WheelsCmdStamped())
        rospy.sleep(stop_time)

    # def drive_backward(self):
    #     self.wheel_cmd.vel_left = -self.forward_speed #* (1 - self.wheel_distance / (2 * self.radius))
    #     self.wheel_cmd.vel_right = -self.forward_speed  #* (1 + self.wheel_distance / (2 * self.radius))
    #     self.pub_wheels.publish(self.wheel_cmd)

    def shutdown_duckie(self):
        rospy.loginfo("Shutting down... stopping the robot.")
        self.line_follow_pub.publish(BoolStamped(data=True))
        rospy.sleep(1)
        self.stop_robot()
        # self.pub_wheels.publish(WheelsCmdStamped())
        # rospy.sleep(1)  # Allow time for the message to propagate
    
    def run(self):
        self.line_follow_pub.publish(BoolStamped(data=False))
        while not rospy.is_shutdown():
            rospy.loginfo("Running...")
            if self.tag_detected:
                self.drive_curve()
            else:
                self.drive_forward()
            self.rate.sleep()
        # rospy.spin()

if __name__ == '__main__':
    try:
        node = MainNode()
        rospy.sleep(1)
        node.run()
    except rospy.ROSInterruptException:
        pass