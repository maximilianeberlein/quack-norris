#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdometryToTF:
    def __init__(self):
        rospy.init_node('odometry_to_tf_broadcaster')
        
        self.tf_broadcaster = TransformBroadcaster()
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
    
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
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(
            translation,
            rotation,
            rospy.Time.now(),
            "base_link",
            "odom"
        )

if __name__ == '__main__':
    try:
        node = OdometryToTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
