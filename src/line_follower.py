#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import BoolStamped

def publish_joystick_override():
    # Initialize the ROS node
    rospy.init_node('joystick_override_publisher', anonymous=True)

    # Define the topic
    vehicle_name = rospy.get_param("~vehicle_name", "olive")
    topic_name = f"/{vehicle_name}/joy_mapper_node/joystick_override"

    # Create a publisher
    pub = rospy.Publisher(topic_name, BoolStamped, queue_size=1)

    # Wait for the publisher to establish
    rospy.sleep(1)

    # Create and populate the message
    # msg = BoolStamped()
    # msg.header.stamp = rospy.Time.now()
    # msg.header.frame_id = ''
    # msg.data = False
    pub.publish(BoolStamped(data=False))

    # # Publish the message
    # rospy.loginfo(f"Publishing to {topic_name}: {msg}")
    # pub.publish(msg)

    # Allow time for message to be sent
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        publish_joystick_override()
    except rospy.ROSInterruptException:
        pass