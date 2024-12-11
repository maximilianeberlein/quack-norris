#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import Int32
import tf
import os

class OdometryNode:
    def __init__(self):
        rospy.init_node('wheel_odometry_node')

        self.bot_name = os.environ.get("VEHICLE_NAME")
        # Parameters
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0335)  # meters
        self.wheel_base = rospy.get_param('~wheel_base', 0.102)  # meters
        self.ticks_per_revolution = rospy.get_param('~ticks_per_revolution', 135)

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()

        # Encoder counts
        self.curr_left_ticks = 0
        self.curr_right_ticks = 0
        self.previous_left_ticks = 0
        self.previous_right_ticks = 0

        # Publishers and Subscribers
        self.odom_pub = rospy.Publisher('/wheel_encoder/odom', Odometry, queue_size=10)
        self.left_ticks_sub = rospy.Subscriber(f'/{self.bot_name}/left_wheel_encoder_node/tick',WheelEncoderStamped, self.left_ticks_callback)
        self.right_ticks_sub = rospy.Subscriber(f'/{self.bot_name}/right_wheel_encoder_node/tick',WheelEncoderStamped, self.right_ticks_callback)

        self.tf_broadcaster = tf.TransformBroadcaster()

    def left_ticks_callback(self, msg):
        self.curr_left_ticks = msg.data

    def right_ticks_callback(self, msg):
        self.curr_right_ticks = msg.data

    def compute_odometry(self):
        # Get current time
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Compute wheel displacements
        left_ticks = self.curr_left_ticks - self.previous_left_ticks
        right_ticks = self.curr_right_ticks - self.previous_right_ticks
        self.previous_left_ticks = self.curr_left_ticks
        self.previous_right_ticks = self.curr_right_ticks
        left_distance = (2 * 3.14159 * self.wheel_radius * left_ticks) / self.ticks_per_revolution
        right_distance = (2 * 3.14159 * self.wheel_radius * right_ticks) / self.ticks_per_revolution

        # Compute linear and angular velocity
        linear_velocity = (left_distance + right_distance) / (2.0 * dt)
        angular_velocity = (right_distance - left_distance) / (self.wheel_base * dt)

        # Update pose
        self.theta += angular_velocity * dt
        self.x += linear_velocity * dt * np.cos(self.theta)
        self.y += linear_velocity * dt * np.sin(self.theta)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.theta))

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        # print('velocity',linear_velocity,angular_velocity)

        self.odom_pub.publish(odom)

        # Broadcast TF
        # self.tf_broadcaster.sendTransform(
        #     (self.x, self.y, 0),
        #     quaternion_from_euler(0, 0, self.theta),
        #     current_time,
        #     "base_link",
        #     "odom"
        # )

    def spin(self):
        rate = rospy.Rate(10)  # 10 Hz
        rospy.wait_for_message(f'/{self.bot_name}/left_wheel_encoder_node/tick', WheelEncoderStamped)
        rospy.wait_for_message(f'/{self.bot_name}/right_wheel_encoder_node/tick', WheelEncoderStamped)
        self.last_time = rospy.Time.now()
        self.previous_left_ticks = self.curr_left_ticks
        self.previous_right_ticks = self.curr_right_ticks
        while not rospy.is_shutdown():
            self.compute_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        odometry_node = OdometryNode()
        odometry_node.spin()
    except rospy.ROSInterruptException:
        pass
