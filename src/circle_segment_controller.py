#!/usr/bin/env python3
import rospy
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped 
from nav_msgs.msg import Odometry
import os
import tf
from std_msgs.msg import Float32, Bool
from quack_norris.msg import CircleSegment

class CircleSegmentController:
    def __init__(self):
        rospy.init_node('circle_segment_controller')
        # publishers
        
        # subscribers
        # self.odom_sub = rospy.Subscriber('/wheel_encoder/odom', Odometry, self.odom_callback)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v_max = 0.8
        self.bot_name = os.environ.get("VEHICLE_NAME")
        self.tf_listener = tf.TransformListener()
        # publishers
        self.wheel_cmd_pub = rospy.Publisher(f'/{self.bot_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        # self.r_error = rospy.Publisher(f'/{self.bot_name}/right_error', Float32, queue_size=1)
        # self.l_error = rospy.Publisher(f'/{self.bot_name}/left_error',Float32, queue_size=1)
        self.takeover_pub = rospy.Publisher(f'/{self.bot_name}/takeover', Bool, queue_size=1)
        # subscribers
        self.desired_wheel_speed_sub = rospy.Subscriber(f'/{self.bot_name}/desired_wheel_speed', WheelsCmdStamped, self.desired_wheel_speed_cb, queue_size=1)
        self.left_ticks_sub = rospy.Subscriber(f'/{self.bot_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_ticks_callback)
        self.right_ticks_sub = rospy.Subscriber(f'/{self.bot_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_ticks_callback)
        self.circle_segment_sub = rospy.Subscriber(f'/{self.bot_name}/circle_segment', CircleSegment, self.circle_segment_cb)
        self.angular_speed_sub = rospy.Subscriber(f'/{self.bot_name}/angular_speed', Float32, self.angular_speed_cb)
        self.desired_angular_speed_sub = rospy.Subscriber(f'/{self.bot_name}/desired_angular_speed', Float32, self.desired_angular_speed_cb)

        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0335)  # meters
        self.l_ticks_per_revolution = rospy.get_param('~ticks_per_revolution', 135)
        self.r_ticks_per_revolution = rospy.get_param('~ticks_per_revolution', 135)
        self.wheel_base = rospy.get_param('~wheel_base', 0.102)


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
        self.speed_previous_error = 0
        self.angular_previous_error = 0
        self.desired_angular_speed = 0  
        self.angular_speed = 0
        self.v_max = 0.8
        self.circle_segment = CircleSegment()
        self.takeover = False
        rospy.on_shutdown(self.shutdown_duckie)
    
    def shutdown_duckie(self):
        rospy.loginfo("Shutting down... stopping the robot.")
        self.wheel_cmd_pub.publish(WheelsCmdStamped())
        rospy.sleep(1)
    def circle_segment_cb(self, msg):
        self.circle_segment = msg
        self.takeover = True
        self.takeover_pub.publish(True)

    def left_ticks_callback(self, msg):
        self.curr_left_ticks = msg.data
        current_time = rospy.Time.now()
        dt = (current_time - self.l_last_time).to_sec()
        dt= np.clip(dt, 0.01, 0.2)
        self.l_last_time = current_time
        left_ticks = self.curr_left_ticks - self.previous_left_ticks
        self.previous_left_ticks = self.curr_left_ticks
        self.l_speed= (2*np.pi*self.wheel_radius*left_ticks)/(self.l_ticks_per_revolution*dt)
    def right_ticks_callback(self, msg):
        self.curr_right_ticks = msg.data
        current_time = rospy.Time.now()
        dt = (current_time - self.r_last_time).to_sec()
        dt= np.clip(dt, 0.01, 0.2)
        self.r_last_time = current_time
        right_ticks = self.curr_right_ticks - self.previous_right_ticks
        self.previous_right_ticks = self.curr_right_ticks
        self.r_speed= (2*np.pi*self.wheel_radius*right_ticks)/(self.r_ticks_per_revolution*dt)


    def angular_speed_cb(self, msg):
        self.angular_speed = msg.data
    def desired_angular_speed_cb(self, msg):
        self.desired_angular_speed = msg.data
    def desired_wheel_speed_cb(self, msg):
        self.desired_wheel_speed(left_speed=msg.vel_left, right_speed=msg.vel_right)
    def desired_wheel_speed(self, left_speed, right_speed):
        # Implement your wheel speed controller here
        # You can access the desired wheel speeds from msg.vel_left and msg.vel_right
        # You can publish the wheel commands using self.wheel_cmd_pub.publish(wheel_msg)
        desired_l_speed = left_speed *self.v_max
        desired_r_speed = right_speed *self.v_max
        # print(f'Desired left speed: {desired_l_speed}, Desired right speed: {desired_r_speed}')
        l_error = Float32(data=desired_l_speed - self.l_speed)
        r_error = Float32(data=desired_r_speed - self.r_speed)
        # self.l_error.publish(l_error)
        # self.r_error.publish(r_error)
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
            # rospy.loginfo(f'Left angular we give: {t_l}, Right angular: {t_r}, Left speed we give: {u_l}, Right speed: {u_r}')
            u_l -= t_l
            u_r += t_r
        wheel_msg = WheelsCmdStamped()
        wheel_msg.vel_left = (desired_l_speed + u_l)/0.8
        wheel_msg.vel_right = (desired_r_speed + u_r)/0.8
        # rospy.loginfo(f'Left speed we give: {wheel_msg.vel_left}, Right speed: {wheel_msg.vel_right}')
        self.wheel_cmd_pub.publish(wheel_msg)
    def pid_correction_speed(self, desired_speed, current_speed):
        kp = 0.4
        ki = 0.0
        kd = 0.01

        if not hasattr(self, 'integral'):
            self.integral = 0
        if not hasattr(self, 'previous_error'):
            self.previous_error = 0

        error = desired_speed - current_speed
        self.speed_integral += error
        self.speed_integral = np.clip(self.speed_integral, -0.3, 0.3)
        derivative = error - self.speed_previous_error

        self.speed_previous_error = error

        return kp * error + ki * self.speed_integral + kd * derivative
    def pid_correction_angular(self, desired_theta_dot, current_theta_dot):
        kp = 0.3
        ki = 0.1
        kd = 0.0

        if not hasattr(self, 'integral'):
            self.integral = 0
        if not hasattr(self, 'previous_error'):
            self.previous_error = 0

        error = desired_theta_dot - current_theta_dot

        error =  np.clip(error, -np.abs(desired_theta_dot), np.abs(desired_theta_dot))
        self.angular_integral += error
        self.angular_integral = np.clip(self.angular_integral, -0.2, 0.2)
        # print(desired_theta_dot, current_theta_dot, error)
        derivative = error - self.angular_previous_error

        self.angular_previous_error = error

        return kp * error + ki * self.angular_integral + kd * derivative
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        # Simple approximation since you're only using z (Not accurate for all orientations)
        # Better: use tf.transformations.euler_from_quaternion
        _, _, self.theta = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    def inverse_kinematics(self,radius,speed):
        L = 0.102
        omega = speed/radius
        speed_r = omega*(radius + L/2)/self.v_max
        speed_l = omega*(radius - L/2)/self.v_max
        return speed_l, speed_r, omega
    def go_to_point(self, x, y, theta, speed):
        wheelbase =0.102
        
        alpha = np.arctan2(y-self.y, x - self.x) - self.theta
        distance = np.sqrt((x - self.x)**2 + (y - self.y)**2)
        steering_angle = np.arctan2(2*wheelbase*np.sin(alpha), distance)


        l_speed = (speed - (steering_angle * wheelbase/2)) 
        r_speed = (speed + (steering_angle * wheelbase/2))
        self.desired_angular_speed = 0
        self.desired_wheel_speed(l_speed, r_speed)
    def get_tf(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base', rospy.Time(0))
            self.x = trans[0]
            self.y = trans[1]
            _, _, self.theta = tf.transformations.euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
    def run(self):
        rate = rospy.Rate(30)
        count=1
        rospy.wait_for_message('/wheel_encoder/odom', Odometry)
        timer = rospy.Time.now()
        while not rospy.is_shutdown():
            self.get_tf()
            
            if self.takeover:
                rospy.loginfo('Taking over')
                self.desired_angular_speed = self.circle_segment.angular_speed
                radius = self.circle_segment.radius

                speed_l = self.circle_segment.left_speed
                speed_r = self.circle_segment.right_speed

                x_end = self.circle_segment.x
                y_end = self.circle_segment.y
                theta_end = self.circle_segment.theta
                goal_theta = (theta_end + np.pi) % (2 * np.pi) - np.pi
                if radius < 100:
                    
                    while np.abs(self.theta - goal_theta) > 0.08:
                        self.get_tf()
                        # rospy.loginfo(f'theta: {self.theta}, goal_theta: {goal_theta}')
                        self.desired_wheel_speed(speed_l, speed_r)
                        
                        rate.sleep()
                    
                    self.takeover = False
                    self.takeover_pub.publish(False)
                    self.desired_wheel_speed(0, 0)

                else:

                    while np.sqrt((self.x - x_end)**2 + (self.y - y_end)**2) > 0.08:
                        self.get_tf()
                        self.go_to_point(x_end, y_end, theta_end,speed_l)
                        rate.sleep()
                    self.takeover = False
                    self.takeover_pub.publish(False)
                    self.desired_wheel_speed(0, 0)
            
            rate.sleep()

if __name__ == '__main__':
    controller = CircleSegmentController()
    controller.run()