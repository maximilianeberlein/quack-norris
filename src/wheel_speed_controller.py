#!/usr/bin/env python3
import rospy
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import Float32

import os


class WheelSpeedControllerNode:
    def __init__(self):
        rospy.init_node('wheel_speed_controller_node')
        self.bot_name = os.environ.get("VEHICLE_NAME")
        # publishers
        self.wheel_cmd_pub = rospy.Publisher(f'/{self.bot_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.r_error = rospy.Publisher(f'/{self.bot_name}/right_error', Float32, queue_size=1)
        self.l_error = rospy.Publisher(f'/{self.bot_name}/left_error',Float32, queue_size=1)
        # subscribers
        self.desired_wheel_speed_sub = rospy.Subscriber(f'/{self.bot_name}/desired_wheel_speed', WheelsCmdStamped, self.desired_wheel_speed_cb, queue_size=1)
        self.left_ticks_sub = rospy.Subscriber(f'/{self.bot_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_ticks_callback)
        self.right_ticks_sub = rospy.Subscriber(f'/{self.bot_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_ticks_callback)
        
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0335)  # meters
        self.ticks_per_revolution = rospy.get_param('~ticks_per_revolution', 135)
        self.wheel_base = rospy.get_param('~wheel_base', 0.102)


        self.curr_left_ticks = 0
        self.curr_right_ticks = 0
        self.previous_left_ticks = 0
        self.previous_right_ticks = 0
        self.l_last_time = rospy.Time.now()
        self.r_last_time = rospy.Time.now()
        self.l_speed = 0
        self.r_speed = 0
        self.integral = 0

    def left_ticks_callback(self, msg):
        self.curr_left_ticks = msg.data
        current_time = rospy.Time.now()
        dt = (current_time - self.l_last_time).to_sec()
        self.l_last_time = current_time
        left_ticks = self.curr_left_ticks - self.previous_left_ticks
        self.previous_left_ticks = self.curr_left_ticks
        self.l_speed= (2*np.pi*self.wheel_radius*left_ticks)/(self.ticks_per_revolution*dt)
    def right_ticks_callback(self, msg):
        self.curr_right_ticks = msg.data
        current_time = rospy.Time.now()
        dt = (current_time - self.r_last_time).to_sec()
        self.r_last_time = current_time
        right_ticks = self.curr_right_ticks - self.previous_right_ticks
        self.previous_right_ticks = self.curr_right_ticks
        self.r_speed= (2*np.pi*self.wheel_radius*right_ticks)/(self.ticks_per_revolution*dt)

    def desired_wheel_speed_cb(self, msg):
        # Implement your wheel speed controller here
        # You can access the desired wheel speeds from msg.vel_left and msg.vel_right
        # You can publish the wheel commands using self.wheel_cmd_pub.publish(wheel_msg)
        desired_l_speed = msg.vel_left
        desired_r_speed = msg.vel_right
        l_error = Float32(data=desired_l_speed - self.l_speed)
        r_error = Float32(data=desired_r_speed - self.r_speed)
        self.l_error.publish(l_error)
        self.r_error.publish(r_error)
        if desired_l_speed == 0 and desired_r_speed == 0:
            u_l = 0
            u_r = 0
        else:
            u_l = self.pid_correction(desired_l_speed, self.l_speed)
            u_r = self.pid_correction(desired_r_speed, self.r_speed)
        wheel_msg = WheelsCmdStamped()
        wheel_msg.vel_left = msg.vel_left + u_l
        wheel_msg.vel_right = msg.vel_right + u_r
        self.wheel_cmd_pub.publish(wheel_msg)
    def pid_correction(self, desired_speed, current_speed):
        kp = 2.0
        ki = 0.1
        kd = 0.01

        if not hasattr(self, 'integral'):
            self.integral = 0
        if not hasattr(self, 'previous_error'):
            self.previous_error = 0

        error = desired_speed - current_speed
        self.integral += error
        derivative = error - self.previous_error

        self.previous_error = error

        return kp * error + ki * self.integral + kd * derivative
    
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    node = WheelSpeedControllerNode()
    node.run()