#!/usr/bin/env python3
import rospy
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped
from nav_msgs.msg import Odometry
import os
from std_msgs.msg import Float32

class CircleSegmentController:
    def __init__(self):
        rospy.init_node('circle_segment_controller')
        # publishers
        self.wheel_cmd_pub = rospy.Publisher(f'/{os.environ.get("VEHICLE_NAME")}/desired_wheel_speed', WheelsCmdStamped, queue_size=1)
        self.angular_speed_pub = rospy.Publisher(f'/{os.environ.get("VEHICLE_NAME")}/desired_angular_speed', Float32, queue_size=1)
        # subscribers
        self.odom_sub = rospy.Subscriber('/wheel_encoder/odom', Odometry, self.odom_callback)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v_max = 0.8
        rospy.on_shutdown(self.shutdown_duckie)
    
    def shutdown_duckie(self):
        rospy.loginfo("Shutting down... stopping the robot.")
        self.wheel_cmd_pub.publish(WheelsCmdStamped())
        rospy.sleep(1)
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.z
    def inverse_kinematics(self,radius,speed):
        L = 0.102
        omega = speed/radius
        speed_r = omega*(radius + L/2)/self.v_max
        speed_l = omega*(radius - L/2)/self.v_max
        return speed_l, speed_r

    def run(self):
        rate = rospy.Rate(10)
        count=1
        rospy.wait_for_message('/wheel_encoder/odom', Odometry)
        while not rospy.is_shutdown():
            radius = 0.5
            speed = 0.2
            speed_l, speed_r = self.inverse_kinematics(radius,speed)
            sector = 15 * count
            rospy.loginfo(f"sector: {sector}")
            sector_rad = np.deg2rad(sector)
            goal_x = self.x + radius*np.cos(sector_rad)
            goal_y = self.y + radius*np.sin(sector_rad)
            goal_theta = self.theta + sector_rad

            while self.theta < goal_theta:
                msg = WheelsCmdStamped()
                msg.header.stamp = rospy.Time.now()
                msg.vel_left = speed_l
                msg.vel_right = speed_r
                self.wheel_cmd_pub.publish(msg)
                rate.sleep()
            count+=1
            
            rospy.sleep(1)
            rate.sleep()
        