#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Point
from tf.transformations import quaternion_from_euler
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import Float32, ColorRGBA
from quack_norris.msg import TagInfo
import shapely.geometry as sg
import tf
import os
import numpy as np
from quack_norris_utils.utils import dubins, SETransform, DuckieObstacle, DuckieNode
from visualization_msgs.msg import Marker

a = 0.585/2
p1 = SETransform(a, a, 3*np.pi/2)
p2 = SETransform(5*a, a, 0)
p3 = SETransform(5*a, 5*a, np.pi/2)
p4 = SETransform(a, 5*a, np.pi)

p1 = DuckieNode(p1, tag_id=58)
p2 = DuckieNode(p2, tag_id=96)
p3 = DuckieNode(p3, tag_id=2)
p4 = DuckieNode(p4, tag_id=20)

p1.insert_next(p2)
p2.insert_next(p3)
p3.insert_next(p4)
p4.insert_next(p1)

p1.insert_parent(p4)
p2.insert_parent(p1)
p3.insert_parent(p2)
p4.insert_parent(p3)

p1.insert_corner(SETransform(2*a,2*a, 7*np.pi/4))
p2.insert_corner(SETransform(4*a,2*a, np.pi/4))
p3.insert_corner(SETransform(4*a,4*a, 3*np.pi/4))
p4.insert_corner(SETransform(2*a,4*a, 5*np.pi/4))
hardcoded_path = [p2, p3, p4, p1]


class DubinsNode:
    def __init__(self):
        rospy.init_node('dubins_node')

        self.bot_name = os.environ.get("VEHICLE_NAME", "duckiebot")
        self.wheel_base = rospy.get_param('~wheel_base', 0.102)
        
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.tag_info_sub = rospy.Subscriber(f'/{self.bot_name}/tag_info', TagInfo, self.tag_info_callback)
        
        self.marker_pub = rospy.Publisher('/dubins_marker', Marker, queue_size=1)
        self.dubins_pub = rospy.Publisher('/dubins_path', Marker, queue_size=1)

        self.se_pose = SETransform(0,0,0)
        self.tag_info = None
        self.path = hardcoded_path
        self.node_lookahead = 2*a
        self.waypoint_lookahead = 1.5*a
        self.next_node = None
        self.node_in_scope = False
        self.tag_present = False
        self.line = None
        self.line_theta = None
        self.duckie_path = None
    def odom_callback(self, msg):
        self.se_pose.x = msg.pose.pose.position.x
        self.se_pose.y = msg.pose.pose.position.y
        # Orientation handling is just an approximation:
        # Assuming small rotations around z. Adjust if you have quaternions:
        # Convert quaternion to yaw if needed:
        q = msg.pose.pose.orientation
        # Simple approximation since you're only using z (Not accurate for all orientations)
        # Better: use tf.transformations.euler_from_quaternion
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.se_pose.theta = yaw

    def tag_info_callback(self, msg):
        self.tag_info = msg

    def get_node_lookahead(self):
        if sg.Point(self.se_pose.x, self.se_pose.y).buffer(self.node_lookahead).contains(sg.Point(self.next_node.x, self.next_node.y)):
            # rospy.loginfo(f"Node at position{self.next_node.x,self.next_node.y} is in scope")
            self.node_in_scope = True
        else:
            self.node_in_scope = False 

    def check_tag(self):
        if self.tag_info is not None:
            # rospy.loginfo(f"Tag {self.tag_info.tag_id} detected, looking for tag {self.next_node.tag_id}")
            if self.next_node.tag_id == self.tag_info.tag_id:
                # rospy.loginfo(f"Tag {self.tag_info.tag_id} is present, we moving on brahh")
                self.tag_present = True
            else:
                self.tag_present = False
        else:
            self.tag_present = False

    def get_line(self):
        self.line = sg.LineString([(self.next_node.parent.x,self.next_node.parent.y), (self.next_node.x, self.next_node.y)])
        self.line_theta = np.arctan2(self.line.coords[1][1] - self.line.coords[0][1], self.line.coords[1][0] - self.line.coords[0][0])

    def get_line_lookahead(self):
        circle = sg.Point(self.se_pose.x, self.se_pose.y).buffer(self.waypoint_lookahead).boundary
        intersection = circle.intersection(self.line)
        rospy.loginfo(f"Intersection: {intersection}")
        if intersection.is_empty:
            return SETransform(self.next_node.x, self.next_node.y, self.line_theta)
        elif intersection.geom_type == 'Point':
            return SETransform(intersection.x, intersection.y, self.line_theta)
       
        # Convert MultiPoint to a list of Points
        elif intersection.geom_type == 'MultiPoint':
        # Find the closest point to the waypoint
            closest_point = min(list(intersection.geoms), key=lambda point: point.distance(sg.Point(self.next_node.x, self.next_node.y)))
            return SETransform(closest_point.x, closest_point.y, self.line_theta)
        else:
            rospy.logwarn(f"Unexpected intersection type: {intersection.geom_type}")
            return SETransform(self.next_node.x, self.next_node.y, self.line_theta)
        
    def publish_markers(self, lookahead_point):
        # Common properties
        frame_id = "map"  # Change this if you have a different fixed frame
        now = rospy.Time.now()

        # Marker for line (representing the path between parent node and next_node)
        line_marker = Marker()
        line_marker.header.frame_id = frame_id
        line_marker.header.stamp = now
        line_marker.ns = "local_planner"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.02  # thickness of the line
        line_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red line

        if self.line:
            # Add the two points defining the line
            line_marker.points = [
                Point(self.line.coords[0][0], self.line.coords[0][1], 0.0),
                Point(self.line.coords[1][0], self.line.coords[1][1], 0.0)
            ]
        
        self.marker_pub.publish(line_marker)

        # Marker for next_node
        if self.next_node:
            next_node_marker = Marker()
            next_node_marker.header.frame_id = frame_id
            next_node_marker.header.stamp = now
            next_node_marker.ns = "local_planner"
            next_node_marker.id = 1
            next_node_marker.type = Marker.SPHERE
            next_node_marker.action = Marker.ADD
            next_node_marker.scale.x = 0.05
            next_node_marker.scale.y = 0.05
            next_node_marker.scale.z = 0.05
            next_node_marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green sphere
            next_node_marker.pose.position.x = self.next_node.x
            next_node_marker.pose.position.y = self.next_node.y
            next_node_marker.pose.position.z = 0.0
            self.marker_pub.publish(next_node_marker)

        # Marker for lookahead point
        if lookahead_point:
            lookahead_marker = Marker()
            lookahead_marker.header.frame_id = frame_id
            lookahead_marker.header.stamp = now
            lookahead_marker.ns = "local_planner"
            lookahead_marker.id = 2
            lookahead_marker.type = Marker.SPHERE
            lookahead_marker.action = Marker.ADD
            lookahead_marker.scale.x = 0.05
            lookahead_marker.scale.y = 0.05
            lookahead_marker.scale.z = 0.05
            lookahead_marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # Blue sphere
            lookahead_marker.pose.position.x = lookahead_point.x
            lookahead_marker.pose.position.y = lookahead_point.y
            lookahead_marker.pose.position.z = 0.0
            self.marker_pub.publish(lookahead_marker)
    def publish_path_markers(self):
        # We'll publish one Marker per segment. Alternatively, you could combine them into one marker.
        # For simplicity, let's publish them all in one go. You can assign each segment its own namespace or ID.
        
        segment_id = 0
        for segment in self.duckie_path:
            marker = Marker()
            marker.header.frame_id = "map"  # Use the appropriate frame, e.g., "odom" or "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "dubins_path"
            marker.id = segment_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Set marker scale and color
            marker.scale.x = 0.02  # thickness of the line in meters
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  #  color, fully opaque

            # Extract coordinates from shapely path
            coords = list(segment.shapely_path.coords)
            for (x, y) in coords:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.0
                marker.points.append(p)

            self.marker_pub.publish(marker)
            segment_id += 1

    def run(self):
        rate = rospy.Rate(10)
        #init by popping the first node 
        self.next_node = self.path.pop(0)

        while not rospy.is_shutdown():
            
            # rospy.loginfo(f'tag id we look_ for = {self.next_node.tag_id}')

            self.get_node_lookahead()
            self.check_tag()

            # If the next_node is in scope and the tag matches, move on to the next node
            if self.node_in_scope and self.tag_present:
                rospy.loginfo(f"Moving to next node")
                self.next_node = self.next_node.next
                
                self.node_in_scope = False
                self.tag_present = False
                self.get_line()  # Update line for new node

            # If we have a next_node but no line yet, compute it
            if self.next_node and (self.line is None):
                self.get_line()

            lookahead_point = self.get_line_lookahead()
            if lookahead_point:
                rospy.loginfo(f"Lookahead Point: x={lookahead_point.x}, y={lookahead_point.y}")
                dub = dubins(self.se_pose,lookahead_point,3,0.3,0.2,0.2)
                self.duckie_path = dub.solve()
                self.publish_path_markers()
            # Publish markers for visualization
            self.publish_markers(lookahead_point)

            rate.sleep()


if __name__ == '__main__':
    node = DubinsNode()
    node.run()
#     rospy.spin()