#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from duckietown_msgs.msg import WheelsCmdStamped
from geometry_msgs.msg import Twist
from quack_norris.msg import TagInfo
from sensor_msgs.msg import Image
import os
import yaml
import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt
import io
import shapely
from PIL import Image as PILImage
from cv_bridge import CvBridge 



class LocalPlannerNode:
    def __init__(self):
        #variables 
        self.load_parameters()

        self.bot_name = os.environ.get("VEHICLE_NAME")

        self.yaml_file = rospy.get_param('~yaml_file', '/code/catkin_ws/src/user_code/quack-norris/params/apriltags.yaml')
        tag_yaml_file = rospy.get_param("~tag_map_file", "/code/catkin_ws/src/user_code/quack-norris/params/tag_map.yaml")
        self.tag_map_raw = self.load_yaml_file(tag_yaml_file)['tags']


        self.bridge = CvBridge()
        #vars_for plot
        self.segment = None
        self.path = None
        self.tag_position = None
        self.tag_id = None 
        self.tag_orientation = None
        #subscribers
        self.tag_info_sub = rospy.Subscriber(f'/{self.bot_name}/tag_info', TagInfo, self.tag_info_cb, queue_size=1)

        #publishers
        self.wheel_cmd_pub = rospy.Publisher(f'/{self.bot_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.image_pub = rospy.Publisher(f'/{self.bot_name}/map_segment', Image, queue_size=1)
    def load_yaml_file(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    def get_map_parameters(self, tag_id):
        for tag in self.tag_map_raw:
            if tag["tag_id"] == tag_id:
                return tag['map_segment'], tag['path']
        return None, None
    
    def tag_info_cb(self, data):
        tag_id = data.tag_id
        self.tag_id = tag_id
        self.tag_position = [data.x, data.y,data.z]
        self.tag_orientation = np.deg2rad(-data.angle)

        self.segment, self.path = self.get_map_parameters(tag_id)
        self.shapely_path =[ shapely.geometry.LineString(self.path[0]), shapely.geometry.LineString(self.path[1])]
        self.shapely_path = self.apply_transform(self.shapely_path, self.tag_position, self.tag_orientation)


        if self.segment is None:
            rospy.logwarn(f"Tag {tag_id} not found in the map")
            
            return
        self.plot_and_publish(tag_id)
        rospy.loginfo(f"Tag {tag_id} found in segment {self.segment} and path {self.path}")

    def apply_transform(self,lines, translation, rotation_angle):
        transformed_lines = []
        for line in lines:
            # Apply translation
            translated_line = shapely.affinity.translate(line, xoff=translation[0], yoff=translation[1])
            # Apply rotation
            rotated_line = shapely.affinity.rotate(translated_line, rotation_angle, origin=(0, 0), use_radians=True)
            transformed_lines.append(rotated_line)
        return transformed_lines
    

    
    def plot_and_publish(self,tag_id):
        # Create a plot
        fig, ax = plt.subplots(figsize=(5, 5))
        ax.axis('equal')
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        for line in self.shapely_path:
            x, y = line.xy
            ax.plot(x, y, label=f"Path {self.segment}")
        ax.plot(self.tag_position[0], self.tag_position[2], marker='o', label=f"Tag {tag_id} Position", color='red')
        ax.set_title(f"Map Segment{self.segment} for Tag {tag_id}")
        ax.set_xlabel("X Coordinate")
        ax.set_ylabel("Y Coordinate")
        ax.legend()
        ax.grid(True)
        plt.tight_layout()

        # Save the plot to a numpy array
        buf = io.BytesIO()
        plt.savefig(buf, format="png")
        buf.seek(0)
        pil_img = PILImage.open(buf)
        cv_img = np.array(pil_img)  # Convert PIL image to numpy array
        buf.close()
        plt.close(fig)

        # Publish the plot as a ROS image
        ros_image = self.bridge.cv2_to_imgmsg(cv_img, encoding="rgba8")
        self.image_pub.publish(ros_image)
        rospy.loginfo(f"Published map segment plot for Tag ID {tag_id}")

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
    
    def controller(self):
        rate = rospy.Rate(10)
        rospy.wait_for_message(f'/{self.bot_name}/tag_info', TagInfo)
        while not rospy.is_shutdown():
            # replace this with actual controller logic
            
            if self.tag_position[0] < -0.10 and self.tag_position[2] < 0.25:
                rospy.loginfo("Reached tag, no go brrrrr")
                self.wheel_cmd_pub.publish(WheelsCmdStamped(header=rospy.Header(), vel_left=0.05, vel_right=0.15))
                
            else:
                rospy.loginfo("Moving to tag")
                self.wheel_cmd_pub.publish(WheelsCmdStamped(header=rospy.Header(), vel_left=0.1, vel_right=0.1))
            rospy.loginfo("Controller running")
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('local_planner_node', anonymous=False)
    node = LocalPlannerNode()
    node.controller()