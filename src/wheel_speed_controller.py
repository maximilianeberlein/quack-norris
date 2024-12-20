#!/usr/bin/env python3

import rospy
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import Float32
import os

class WheelSpeedControllerNode:
    """
    This node implements a simple PID wheel speed controller for the Duckiebot.
    """
    def __init__(self):
        rospy.init_node('wheel_speed_controller_node')
        self.bot_name = os.environ.get("VEHICLE_NAME")

        # Publishers
        self.wheel_cmd_pub = rospy.Publisher(f'/{self.bot_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.r_error = rospy.Publisher(f'/{self.bot_name}/right_error', Float32, queue_size=1)
        self.l_error = rospy.Publisher(f'/{self.bot_name}/left_error',Float32, queue_size=1)

        # Subscribers
        self.desired_wheel_speed_sub = rospy.Subscriber(f'/{self.bot_name}/desired_wheel_speed', WheelsCmdStamped, self.desired_wheel_speed_callback, queue_size=1)
        self.left_ticks_sub = rospy.Subscriber(f'/{self.bot_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_ticks_callback)
        self.right_ticks_sub = rospy.Subscriber(f'/{self.bot_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_ticks_callback)
        self.angular_speed_sub = rospy.Subscriber(f'/{self.bot_name}/angular_speed', Float32, self.angular_speed_callback)
        self.desired_angular_speed_sub = rospy.Subscriber(f'/{self.bot_name}/desired_angular_speed', Float32, self.desired_angular_speed_callback)

        # Parameters
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0335)  # meters
        self.l_ticks_per_revolution = rospy.get_param('~ticks_per_revolution', 135)
        self.r_ticks_per_revolution = rospy.get_param('~ticks_per_revolution', 135)
        self.wheel_base = rospy.get_param('~wheel_base', 0.102)

        # Variables
        self.curr_left_ticks = 0
        self.curr_right_ticks = 0
        self.previous_left_ticks = 0
        self.previous_right_ticks = 0
        self.l_last_time = rospy.Time.now()
        self.r_last_time = rospy.Time.now()
        self.l_speed = 0
        self.r_speed = 0
        self.speed_integral = 0
        self.speed_derivative = 0
        self.angular_integral = 0
        self.angular_derivative = 0
        self.l_last_time = rospy.Time.now()
        self.r_last_time = rospy.Time.now()
        self.speed_previous_error = 0
        self.angular_previous_error = 0
        self.desired_angular_speed = 0  
        self.angular_speed = 0
        self.v_max = 0.8

        # Shutdown
        rospy.on_shutdown(self.shutdown_duckie)
    
    def shutdown_duckie(self):
        rospy.loginfo("Shutting down... stopping the robot.")
        self.wheel_cmd_pub.publish(WheelsCmdStamped())
        rospy.sleep(1)

    def left_ticks_callback(self, msg) -> None:
        """
        Callback for the left wheel encoder. Calculates the angular speed of the left wheel, given the number of ticks.
        """
        self.curr_left_ticks = msg.data

        # Calculate time difference "dt"
        current_time = rospy.Time.now()
        dt = (current_time - self.l_last_time).to_sec()
        dt = np.clip(dt, 0.01, 0.2)
        self.l_last_time = current_time

        # Calculate angular speed of the left wheel
        left_ticks = self.curr_left_ticks - self.previous_left_ticks
        self.previous_left_ticks = self.curr_left_ticks
        self.l_speed = (2*np.pi*self.wheel_radius*left_ticks)/(self.l_ticks_per_revolution*dt)

    def right_ticks_callback(self, msg) -> None:
        """
        Callback for the right wheel encoder. Calculates the angular speed of the right wheel, given the number of ticks.
        """
        self.curr_right_ticks = msg.data

        # Calculate time difference "dt"
        current_time = rospy.Time.now()
        dt = (current_time - self.r_last_time).to_sec()
        dt = np.clip(dt, 0.01, 0.2)
        self.r_last_time = current_time

        # Calculate angular speed of the right wheel
        right_ticks = self.curr_right_ticks - self.previous_right_ticks
        self.previous_right_ticks = self.curr_right_ticks
        self.r_speed = (2*np.pi*self.wheel_radius*right_ticks)/(self.r_ticks_per_revolution*dt)

    def angular_speed_callback(self, msg) -> None:
        self.angular_speed = msg.data

    def desired_angular_speed_callback(self, msg) -> None:
        self.desired_angular_speed = msg.data

    def desired_wheel_speed_callback(self, msg) -> None:
        """
        Callback for the desired wheel speed. Calculates the error between the 
        desired and actual speed and applies a PID controller to correct it.
        """
        desired_l_speed = msg.vel_left *self.v_max
        desired_r_speed = msg.vel_right *self.v_max
        l_error = Float32(data=desired_l_speed - self.l_speed)
        r_error = Float32(data=desired_r_speed - self.r_speed)
        self.l_error.publish(l_error)
        self.r_error.publish(r_error)

        # Calculate the PID-corrected wheel speeds
        if desired_l_speed == 0 and desired_r_speed == 0:
            u_l = 0
            u_r = 0
        else:
            u_l = self.pid_correction_speed(desired_l_speed, self.l_speed)
            u_r = self.pid_correction_speed(desired_r_speed, self.r_speed)
            if self.desired_angular_speed > 0:
                t_l = self.pid_correction_angular(self.desired_angular_speed, self.angular_speed)
                t_r = self.pid_correction_angular(self.desired_angular_speed, self.angular_speed)
            else:
                t_l= 0
                t_r= 0
            u_l -= t_l
            u_r += t_r
        
        # Publish the corrected wheel speeds
        wheel_msg = WheelsCmdStamped()
        wheel_msg.vel_left = (desired_l_speed + u_l)/0.8
        wheel_msg.vel_right = (desired_r_speed + u_r)/0.8
        self.wheel_cmd_pub.publish(wheel_msg)

    def pid_correction_speed(self, desired_speed: float, current_speed: float) -> float:
        """
        PID controller to correct the wheel speed.
        """
        kp = 0.4
        ki = 0.0
        kd = 0.01

        if not hasattr(self, 'integral'):
            self.integral = 0
        if not hasattr(self, 'previous_error'):
            self.previous_error = 0

        # Calculate the error (p-part)
        error = desired_speed - current_speed          
        # Calculate the error integral (i-part) and clip it (anti-windup)
        self.speed_integral += error                                    
        self.speed_integral = np.clip(self.speed_integral, -0.3, 0.3)
        # Calculate the error derivative (d-part)
        derivative = error - self.speed_previous_error

        self.speed_previous_error = error

        # Return the PID-corrected speed
        return kp * error + ki * self.speed_integral + kd * derivative
    
    def pid_correction_angular(self, desired_theta_dot: float, current_theta_dot: float) -> float:
        """
        PID controller to correct the angular speed.
        """
        kp = 0.2
        ki = 0.1
        kd = 0.0

        if not hasattr(self, 'integral'):
            self.integral = 0
        if not hasattr(self, 'previous_error'):
            self.previous_error = 0

        # Calculate the error (p-part) and clip it
        error = desired_theta_dot - current_theta_dot
        error = np.clip(error, -np.abs(desired_theta_dot), np.abs(desired_theta_dot))
        # Calculate the error integral (i-part) and clip it (anti-windup)
        self.angular_integral += error
        self.angular_integral = np.clip(self.angular_integral, -0.2, 0.2)
        # Calculate the error derivative (d-part)
        derivative = error - self.angular_previous_error

        self.angular_previous_error = error

        # Return the PID-corrected angular speed
        return kp * error + ki * self.angular_integral + kd * derivative
    
    def run(self):
        rate = rospy.Rate(20)
        rospy.wait_for_message(f'/{self.bot_name}/desired_wheel_speed', WheelsCmdStamped)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    node = WheelSpeedControllerNode()
    node.run()