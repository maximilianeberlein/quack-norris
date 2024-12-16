#!/usr/bin/env python3
import rospy
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped
from nav_msgs.msg import Odometry
import os
import tf
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

    def run(self):
        rate = rospy.Rate(30)
        count=1
        rospy.wait_for_message('/wheel_encoder/odom', Odometry)
        timer = rospy.Time.now()
        while not rospy.is_shutdown():
            
            
            if (rospy.Time.now() - timer).to_sec() < 45:
                radius = 0.1
                speed = 0.2
            elif (rospy.Time.now() - timer).to_sec() < 90:
                count= 1
                radius = 0.3
                speed = 0.4
            elif (rospy.Time.now() - timer).to_sec() < 135:
                count= 1
                radius = 0.5
                speed = 0.6
                
            else:
                timer = rospy.Time.now()
                count = 1
            speed_l, speed_r, omega = self.inverse_kinematics(radius,speed)
            omega = -omega
            sector = 5 * count
            rospy.loginfo(f"sector: {sector}")
            sector_rad = np.deg2rad(sector)
            goal_x = self.x + radius*np.cos(sector_rad)
            goal_y = self.y + radius*np.sin(sector_rad)
            goal_theta = self.theta + sector_rad
            goal_theta = (goal_theta + np.pi) % (2 * np.pi) - np.pi
            rospy.loginfo(f"theta {self.theta} goal_theta: {goal_theta}")
            while np.abs(self.theta - goal_theta) > 0.05:
                msg = WheelsCmdStamped()
                msg.header.stamp = rospy.Time.now()
                msg.vel_left = -speed_l
                msg.vel_right = -speed_r

                self.wheel_cmd_pub.publish(msg)
                self.angular_speed_pub.publish(omega)
                rate.sleep()
            msg = WheelsCmdStamped()
            msg.header.stamp = rospy.Time.now()
            msg.vel_left = 0
            msg.vel_right = 0
            self.wheel_cmd_pub.publish(msg)
            self.angular_speed_pub.publish(0)
            rospy.sleep(1)
            count+=1
            
            
            rate.sleep()

if __name__ == '__main__':
    controller = CircleSegmentController()
    controller.run()