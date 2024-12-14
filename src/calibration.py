#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
import tf
import os
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Float32
import matplotlib.pyplot as plt



class Calibration:
    def __init__(self):
        rospy.init_node('calibration_node')

        self.bot_name = os.environ.get("VEHICLE_NAME")
        # Parameters
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0335)  # meters
        self.wheel_base = rospy.get_param('~wheel_base', 0.102)  # meters
        self.ticks_per_revolution_left = rospy.get_param('~ticks_per_revolution_left', 135)
        self.ticks_per_revolution_right = rospy.get_param('~ticks_per_revolution_right', 135)

        # State
        self.x = 0.585
        self.y = 0.585/4
        self.theta = 0.0
        self.last_time = rospy.Time.now()

        # Encoder counts
        self.curr_left_ticks = 0
        self.curr_right_ticks = 0
        self.previous_left_ticks = 0
        self.previous_right_ticks = 0
        
        self.imu_yaw_vel_array = []
        self.imu_yaw_vel = 0.0
        # Publishers and Subscribers
        self.odom_sub = rospy.Subscriber('/wheel_encoder/odom', Odometry, self.odom_callback)
        self.wheel_cmd_pub = rospy.Publisher(f'/{self.bot_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.desired_wheel_cmd_pub = rospy.Publisher(f'/{self.bot_name}/desired_wheel_speed', WheelsCmdStamped, queue_size=1)
        self.desired_angular_speed = rospy.Publisher(f'/{self.bot_name}/desired_angular_speed', Float32, queue_size=1)
        self.left_speed_sub = rospy.Subscriber(f'/{self.bot_name}/left_speed', Float32, self.left_speed_callback)
        self.right_speed_sub = rospy.Subscriber(f'/{self.bot_name}/right_speed', Float32, self.right_speed_callback)
        self.left_speed = []
        self.right_speed = []
        #self.r_error = rospy.Publisher(f'/{self.bot_name}/right_speed', Float32, queue_size=1)
        # self.l_error = rospy.Publisher(f'/{self.bot_name}/left_speed',Float32, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.x = 0
        self.y = 0
        self.theta = 0

        self.desired_speed_actual = 0.2
        self.v_max = 0.8
        rospy.on_shutdown(self.shutdown_duckie)
    def shutdown_duckie(self):
        rospy.loginfo("Shutting down... stopping the robot.")
        self.wheel_cmd_pub.publish(WheelsCmdStamped())
        rospy.sleep(1)  # A   
    def left_speed_callback(self, msg):
        self.left_speed.append(msg.data)
    def right_speed_callback(self, msg):
        self.right_speed.append(msg.data)
    
    
    def odom_callback(self, msg):
        # Set translation
        translation = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        
        # Set rotation (quaternion)
        rotation = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        
        self.x = translation[0]
        self.y = translation[1]
        self.theta = rotation[2]

    def run(self):
        rate = rospy.Rate(10)
        rospy.wait_for_message('/wheel_encoder/odom', Odometry)
        timer = rospy.Time.now()
        while not rospy.is_shutdown():
            if rospy.Time.now() - timer > rospy.Duration(20):
                wheel_cmd = WheelsCmdStamped()
                wheel_cmd.header.stamp = rospy.Time.now()
                wheel_cmd.vel_left = 0
                wheel_cmd.vel_right = 0
                self.wheel_cmd_pub.publish(wheel_cmd)
                plt.plot(self.left_speed)
    
                plt.axhline(y=self.desired_speed_actual, color='r', linestyle='--')
                plt.grid()

                plt.plot(self.right_speed)
                plt.savefig('/code/catkin_ws/src/user_code/quack-norris/plots/calibration.png')
                rospy.signal_shutdown("Reached target position")
            elif rospy.Time.now() - timer > rospy.Duration(5):
                wheel_cmd = WheelsCmdStamped()
                wheel_cmd.header.stamp = rospy.Time.now()
                wheel_cmd.vel_left = 0.249/self.v_max
                wheel_cmd.vel_right = 0.351/self.v_max
                self.desired_wheel_cmd_pub.publish(wheel_cmd)
                self.desired_angular_speed.publish(Float32(data=1.0))
                
                
            
                
            rate.sleep()
if __name__ == '__main__':
    node = Calibration()
    node.run()

