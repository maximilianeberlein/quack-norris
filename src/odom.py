#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import tf
import os
import numpy as np

class OdometryNode:
    """
    This node computes the odometry of the Duckiebot using wheel encoders and IMU data.
    """
    def __init__(self):
        rospy.init_node('wheel_odometry_node')
        self.bot_name = os.environ.get("VEHICLE_NAME")

        # Parameters
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0375)    # meters
        self.wheel_base = rospy.get_param('~wheel_base', 0.102)         # meters
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

        # Subscribers
        self.odom_pub = rospy.Publisher('/wheel_encoder/odom', Odometry, queue_size=1)
        self.imu_sub = rospy.Subscriber(f'/{self.bot_name}/imu_node/data', Imu, self.imu_callback)
        self.left_ticks_sub = rospy.Subscriber(f'/{self.bot_name}/left_wheel_encoder_node/tick',WheelEncoderStamped, self.left_ticks_callback)
        self.right_ticks_sub = rospy.Subscriber(f'/{self.bot_name}/right_wheel_encoder_node/tick',WheelEncoderStamped, self.right_ticks_callback)
        self.get_global_pose = rospy.Subscriber(f'/duckiebot_globalpose', PoseWithCovarianceStamped, self.global_pose_callback)

        # Publishers
        self.r_error = rospy.Publisher(f'/{self.bot_name}/right_speed', Float32, queue_size=1)
        self.l_error = rospy.Publisher(f'/{self.bot_name}/left_speed',Float32, queue_size=1)
        self.angular_speed = rospy.Publisher(f'/{self.bot_name}/angular_speed',Float32, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()

    def imu_callback(self, msg):
        """
        Callback function for the IMU data.
        A running average of the yaw velocity is computed to counteract noise.
        The average is not done over long periods of time, as the yaw velocity vector is reset in "compute_odometry()".
        """
        rot_vel = msg.angular_velocity
        yaw_vel = rot_vel.z

        # Running average
        self.imu_yaw_vel_array.append(yaw_vel)
        self.imu_yaw_vel = np.mean(self.imu_yaw_vel_array)
        
    def left_ticks_callback(self, msg):
        self.curr_left_ticks = msg.data

    def right_ticks_callback(self, msg):
        self.curr_right_ticks = msg.data

    def global_pose_callback(self, msg):
        """
        Callback function for the global pose (x, y, theta) of the Duckiebot.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]

        # Convert quaternion to Euler angles
        _, _, yaw = euler_from_quaternion(quaternion)
        self.theta = yaw
                
    def compute_odometry(self):
        """
        Compute and publish the odometry of the Duckiebot.
        """
        # Calculate time difference "dt"
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Publish left wheel velocity
        left_ticks = self.curr_left_ticks - self.previous_left_ticks
        self.previous_left_ticks  = self.curr_left_ticks
        left_distance = (2 * np.pi * self.wheel_radius * left_ticks) / self.ticks_per_revolution_left
        l_speed = Float32(left_distance/dt)
        self.l_error.publish(l_speed)

        # Publish right wheel velocity
        right_ticks = self.curr_right_ticks - self.previous_right_ticks
        self.previous_right_ticks = self.curr_right_ticks
        right_distance = (2 * np.pi * self.wheel_radius * right_ticks) / self.ticks_per_revolution_right
        r_speed = Float32(right_distance/dt)
        self.r_error.publish(r_speed)
        
        # Total velocity
        linear_velocity = (left_distance + right_distance) / (2.0 * dt)

        # Publish angular velocity
        angular_velocity = self.imu_yaw_vel
        self.angular_speed.publish(Float32(angular_velocity))
        self.imu_yaw_vel_array = [] # Reset the array used for the running average

        # Update pose
        self.theta += angular_velocity * dt
        self.x += linear_velocity * dt * np.cos(self.theta)
        self.y += linear_velocity * dt * np.sin(self.theta)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "map"
        odom.child_frame_id = "base"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.theta))

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom)

        # Broadcast TF
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0),
            quaternion_from_euler(0, 0, self.theta),
            current_time,
            "base",
            "odom"
        )

    def spin(self):
        # Init
        rate = rospy.Rate(20)
        rospy.wait_for_message(f'/{self.bot_name}/left_wheel_encoder_node/tick', WheelEncoderStamped)
        rospy.wait_for_message(f'/{self.bot_name}/right_wheel_encoder_node/tick', WheelEncoderStamped)

        # Reset variables
        self.last_time = rospy.Time.now()
        self.previous_left_ticks = self.curr_left_ticks
        self.previous_right_ticks = self.curr_right_ticks

        # Run
        while not rospy.is_shutdown():
            self.compute_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        odometry_node = OdometryNode()
        odometry_node.spin()
    except rospy.ROSInterruptException:
        pass
