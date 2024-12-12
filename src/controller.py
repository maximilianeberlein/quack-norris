#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from duckietown_msgs.msg import WheelsCmdStamped
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from quack_norris.msg import TagInfo
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import os

class SETransform:
    def __init__(self,x:float,y:float,theta:float):
        self.theta = theta
        self.x = x
        self.y = y

class ControllerNode:
    def __init__(self):
        rospy.init_node('controller_node')
        self.bot_name = os.environ.get("VEHICLE_NAME")
        self.listener = tf.TransformListener()
        self.path = []
        self.done = False
        self.got_path = False
        self.speed = 0
        #publishers
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        self.wheel_cmd_pub = rospy.Publisher(f'/{self.bot_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.desired_wheel_speed_pub = rospy.Publisher(f'/{self.bot_name}/desired_wheel_speed', WheelsCmdStamped, queue_size=1)
        #subscribers
        self.odom_sub = rospy.Subscriber(f'/wheel_encoder/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.on_shutdown(self.shutdown_duckie)
    def odom_cb(self, msg):
        self.speed = msg.twist.twist.linear.x
    def path_generator(self,pose):
    #lets do a corner with radius 0.5 
        dt = 0.1
        speed = 0.2
        first_dist = 0.585
        sector = np.pi/2
        radius = 0.585
        second_dist = 0.585
        f_steps = int(first_dist/speed/dt)
        part_1_x  = np.linspace(pose.x, pose.x + first_dist*np.cos(pose.theta), f_steps)
        part_1_y = np.linspace(pose.y, pose.y + first_dist*np.sin(pose.theta), f_steps)
        part_1_theta = np.ones(f_steps)*pose.theta

        s_start_angle = part_1_theta[-1] 
        s_end_angle = s_start_angle + sector
        s_steps = int(sector*radius/speed/dt)
        if s_start_angle < 0:
            s_start_angle += 2*np.pi
        if s_end_angle < 0:
            s_end_angle += 2*np.pi

        angle = np.linspace(s_start_angle, s_end_angle, s_steps)

        s_x = part_1_x[-1] + radius*np.cos(s_start_angle + np.pi/2) + radius*np.cos(angle - np.pi/2)
        s_y = part_1_y[-1] + radius*np.sin(s_start_angle + np.pi/2) + radius*np.sin(angle - np.pi/2)
        s_theta = angle

        part_2_x = np.linspace(s_x[-1], s_x[-1] + second_dist*np.cos(s_theta[-1]), int(second_dist/speed/dt))
        part_2_y = np.linspace(s_y[-1], s_y[-1] + second_dist*np.sin(s_theta[-1]), int(second_dist/speed/dt))
        part_2_theta = np.ones(len(part_2_x))*s_theta[-1]
        x = np.concatenate((part_1_x, s_x, part_2_x))
        y = np.concatenate((part_1_y, s_y, part_2_y))
        theta = np.concatenate((part_1_theta, s_theta, part_2_theta))
        return np.vstack([x, y, theta]).T

    def publish_path_markers(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Point width
        marker.scale.y = 0.1  # Point height
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for position in self.path:
            point = Point()
            point.x = position[0]
            point.y = position[1]
            point.z = 0.0  # Assuming 2D path, set z to 0
            marker.points.append(point)

        self.marker_pub.publish(marker)
    
    def get_position(self):
        try:
            (trans, rot) = self.listener.lookupTransform('/odom', '/base', rospy.Time(0))
 
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF Exception")
            return None, None
    
    def pure_pursuit_control(self, path, lookahead_distance, wheelbase, speed):
    # Extract the path points
        x_path = path[:, 0]
        y_path = path[:, 1]
        theta_path = path[:, 2]

        # Find the closest point on the path
        distances = np.sqrt((x_path - self.pose.x)**2 + (y_path - self.pose.y)**2)
        closest_index = np.argmin(distances)

        # Find the lookahead point
        lookahead_index = closest_index
        while lookahead_index < len(path) and distances[lookahead_index] < lookahead_distance:
            lookahead_index += 1

        if lookahead_index >= len(path):
            lookahead_index = len(path) - 1

            rospy.loginfo("Reached the end of the path")

        lookahead_point = path[lookahead_index]

        # Calculate the steering angle
        gain = 1 #speed /np.clip(self.speed,0.001,10)
        gain = np.clip(gain, 0.5, 5)
        angle_delta = lookahead_point[2] - self.pose.theta
        angle_gain = np.abs(angle_delta)*1.5+1
        angle_gain = np.clip(angle_gain, 1, 2)
        alpha = np.arctan2(lookahead_point[1] - self.pose.y, lookahead_point[0] - self.pose.x) - self.pose.theta
        steering_angle = np.arctan2(2 * wheelbase * np.sin(alpha), lookahead_distance)*angle_gain
        


        # Calculate the left and right wheel speeds

        
        
        l_speed = (speed - (steering_angle * wheelbase / 2))*gain/0.75
        r_speed = (speed + (steering_angle * wheelbase / 2))*gain/0.75

        return l_speed, r_speed
    
    def shutdown_duckie(self):
        rospy.loginfo("Shutting down... stopping the robot.")
        self.wheel_cmd_pub.publish(WheelsCmdStamped())
        rospy.sleep(1)  # Allow time for the message to propagate
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            position, orientation = self.get_position()
            if position is None or orientation is None:
                rate.sleep()
                continue
            self.pose = SETransform(position[0], position[1], orientation[2])
            if len(self.path) ==0:
                self.path = self.path_generator(self.pose)
                self.publish_path_markers()
            if np.sqrt((self.path[-1,0] - self.pose.x)**2 + (self.path[-1,1]- self.pose.y)**2) < 0.2:
                self.done = True
            if self.done:
                rospy.loginfo("Done for the day")
                self.path = None
                
            l_speed, r_speed = self.pure_pursuit_control(self.path, 0.04, 0.102, 0.2)
            if self.pose.x> 1.5:
                self.done = True
            if not self.done:
                wheels_cmd = WheelsCmdStamped()
                wheels_cmd.header.stamp = rospy.Time.now()
                wheels_cmd.vel_left = l_speed
                wheels_cmd.vel_right = r_speed
                #self.wheel_cmd_pub.publish(wheels_cmd)
                
            else:
                wheels_cmd = WheelsCmdStamped()
                wheels_cmd.header.stamp = rospy.Time.now()
                wheels_cmd.vel_left = 0
                wheels_cmd.vel_right =0
                rospy.loginfo(f"Left speed: {l_speed}, Right speed: {r_speed}")
                #self.wheel_cmd_pub.publish(wheels_cmd)
            # wheels_cmd = WheelsCmdStamped()
            # wheels_cmd.header.stamp = rospy.Time.now()
            # wheels_cmd.vel_left = 1
            # wheels_cmd.vel_right =1
            self.wheel_cmd_pub.publish(wheels_cmd)
            #self.desired_wheel_speed_pub.publish(wheels_cmd)
            rate.sleep()
        
if __name__ == '__main__':
    node = ControllerNode()
    node.run()