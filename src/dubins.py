#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Point
from tf.transformations import quaternion_from_euler
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Float32, ColorRGBA
from quack_norris.msg import TagInfo
import shapely.geometry as sg
import tf
from typing import List
import os
import numpy as np

from quack_norris_utils.utils import dubins, SETransform, DuckieObstacle, DuckieNode, DuckieSegment, DuckieCorner
from visualization_msgs.msg import Marker

a = 0.585/4
p1 = SETransform(a, a, 3*np.pi/2)
p2 = SETransform(19*a, a, 0)
p3 = SETransform(19*a, 11*a, np.pi/2)
p4 = SETransform(a, 11*a, np.pi)

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

p1.insert_corner(DuckieCorner(SETransform(4*a,4*a, 7*np.pi/4),3*a,'LEFT'))
p2.insert_corner(DuckieCorner(SETransform(16*a,4*a, np.pi/4),3*a,'LEFT'))
p3.insert_corner(DuckieCorner(SETransform(16*a,8*a, 3*np.pi/4),2*a,'LEFT'))
p4.insert_corner(DuckieCorner(SETransform(4*a,8*a, 5*np.pi/4),1*a,'LEFT'))
hardcoded_path = [p2, p3, p4, p1]


class DubinsNode:
    def __init__(self):
        rospy.init_node('dubins_node')

        self.bot_name = os.environ.get("VEHICLE_NAME", "duckiebot")
        self.wheel_base = rospy.get_param('~wheel_base', 0.102)
        
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.tag_info_sub = rospy.Subscriber(f'/{self.bot_name}/tag_info', TagInfo, self.tag_info_callback, buff_size = 1)
        
        self.marker_pub = rospy.Publisher('/dubins_marker', Marker, queue_size=1)
        self.dubins_pub = rospy.Publisher('/dubins_path', Marker, queue_size=1)
        self.wheel_cmd_pub = rospy.Publisher(f'/{self.bot_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.se_pose = SETransform(0,0,0)
        self.tag_info = None
        self.tag_distance = 10
        self.path = hardcoded_path
        self.node_lookahead = 6*a
        self.waypoint_lookahead = 8*a
        self.next_node = None
        self.node_in_scope = False
        self.tag_present = False
        self.line = None
        self.line_theta = None
        self.duckie_path = None
        self.pursuit_path = None
        self.running_dubs = False
        self.do_dubins = False
        self.corner = None
        rospy.on_shutdown(self.shutdown_duckie)
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
        self.tag_distance = np.sqrt(self.tag_info.x**2 + self.tag_info.y**2)

    def get_node_lookahead(self):
        if sg.Point(self.se_pose.x, self.se_pose.y).buffer(self.node_lookahead).contains(sg.Point(self.next_node.x, self.next_node.y)):
            # rospy.loginfo(f"Node at position{self.next_node.x,self.next_node.y} is in scope")
            self.node_in_scope = True
        else:
            self.node_in_scope = False 

    def check_tag(self):
        if self.tag_info is not None:
            # rospy.loginfo(f"Tag {self.tag_info.tag_id} detected, looking for tag {self.next_node.tag_id}")
            if self.next_node.tag_id == self.tag_info.tag_id and self.tag_distance < 1.5*a:
                #rospy.loginfo(f"Tag {self.tag_info.tag_id} is present, we moving on brahh")
                self.tag_present = True
            
            else:
                self.tag_present = False
        else:
            self.tag_present = False
    
    def check_collision(self,obstacles: List[sg.Polygon], duckie_path : List[DuckieSegment]):
        collision = False
        for segment in duckie_path:
            for obstacle in obstacles:
                if segment.shapely_path.intersects(obstacle):
                    print(f"Collision detected with obstacle {obstacle}")
                    collision = True
                    break
        return collision
    
    def extend_line(self,line, distance):
        """
        Extend a LineString by a given distance along its last segment.
        """
        if line.is_empty or len(line.coords) < 2:
            raise ValueError("LineString must have at least two points to extend.")
        
        # Get the last two points
        x1, y1 = line.coords[-2]
        x2, y2 = line.coords[-1]
        
        # Calculate the direction vector (dx, dy)
        dx = x2 - x1
        dy = y2 - y1
        
        # Normalize the direction vector
        length = np.sqrt(dx**2 + dy**2)
        dx /= length
        dy /= length
        
        # Calculate the new point by extending in the direction of (dx, dy)
        new_x = x2 + dx * distance
        new_y = y2 + dy * distance
        
        # Create a new LineString with the extended point
        new_coords = list(line.coords) + [(new_x, new_y)]
        return sg.LineString(new_coords)

    def get_line(self):
        self.line = sg.LineString([(self.next_node.parent.x,self.next_node.parent.y), (self.next_node.x, self.next_node.y)])
        self.line = self.extend_line(self.line, 0.5)
        self.line_theta = np.arctan2(self.line.coords[1][1] - self.line.coords[0][1], self.line.coords[1][0] - self.line.coords[0][0])
    def decrease_lookahead(self,pose: SETransform, distance: float):
        
        new_x = pose.x + np.cos(pose.theta)*distance
        new_y = pose.y + np.sin(pose.theta)*distance

        return SETransform(new_x, new_y, pose.theta)

    def get_line_lookahead(self):
        circle = sg.Point(self.se_pose.x, self.se_pose.y).buffer(self.waypoint_lookahead).boundary
        intersection = circle.intersection(self.line)
        # rospy.loginfo(f"Intersection: {intersection}")
        if intersection.is_empty:
            return SETransform(self.next_node.x, self.next_node.y, self.line_theta)
        elif intersection.geom_type == 'Point':
            intersect = SETransform(intersection.x, intersection.y, self.line_theta)
            return self.decrease_lookahead(intersect,-0.2)
            
       
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
    
    def pure_pursuit_control(self, path, lookahead_distance, wheelbase, speed):
    # Extract the path points
        x_path = path[:, 0]
        y_path = path[:, 1]
        theta_path = path[:, 2]

        # Find the closest point on the path
        distances = np.sqrt((x_path - self.se_pose.x)**2 + (y_path - self.se_pose.y)**2)
        closest_index = np.argmin(distances)

        # Find the lookahead point
        lookahead_index = closest_index
        while lookahead_index < len(path) and distances[lookahead_index] < lookahead_distance:
            lookahead_index += 1

        if lookahead_index >= len(path):
            lookahead_index = len(path) - 1

            rospy.loginfo("Reached the end of the path")

        lookahead_point = path[lookahead_index]

        # Calculate the steering angle
        gain = 1 #speed /np.clip(self.speed,0.001,10)
        gain = np.clip(gain, 0.5, 5)
        angle_delta = lookahead_point[2] - self.se_pose.theta
        angle_gain = np.abs(angle_delta)*1.5+1
        angle_gain = np.clip(angle_gain, 1, 2)
        alpha = np.arctan2(lookahead_point[1] - self.se_pose.y, lookahead_point[0] - self.se_pose.x) - self.se_pose.theta
        steering_angle = np.arctan2(2 * wheelbase * np.sin(alpha), lookahead_distance)*angle_gain
        


        # Calculate the left and right wheel speeds

        
        
        l_speed = (speed - (steering_angle * wheelbase / 2))*gain/0.75
        r_speed = (speed + (steering_angle * wheelbase / 2))*gain/0.75

        return l_speed, r_speed

    def check_completion(self):
        dist_to_end = np.sqrt((self.se_pose.x - self.pursuit_path[-1,0])**2 + (self.se_pose.y - self.pursuit_path[-1,1])**2)
        if dist_to_end < 0.2:
            rospy.loginfo("Path completed")
            self.running_dubs = False
    def shutdown_duckie(self):
        rospy.loginfo("Shutting down... stopping the robot.")
        self.wheel_cmd_pub.publish(WheelsCmdStamped())
        rospy.sleep(1)  # A   
    def run(self):
        rate = rospy.Rate(10)
        #init by popping the first node 
        self.next_node = self.path.pop(0)
        rospy.wait_for_message('/odometry/filtered', Odometry)
        while not rospy.is_shutdown():
            
            # rospy.loginfo(f'tag id we look_ for = {self.next_node.tag_id}')

            self.get_node_lookahead()
            self.check_tag()

            # If the next_node is in scope and the tag matches, move on to the next node
            if self.node_in_scope and self.tag_present:
                rospy.loginfo(f"Moving to next node")
                self.corner = self.next_node.corner
                self.next_node = self.next_node.next
                self.do_dubins = True
                self.node_in_scope = False
                self.tag_present = False
                self.get_line()  # Update line for new node
                

            # If we have a next_node but no line yet, compute it
            if self.next_node and self.line is None:
                self.get_line()

            lookahead_point = self.get_line_lookahead()
            if lookahead_point and self.do_dubins and not self.running_dubs:
                # rospy.loginfo(f"Lookahead Point: x={lookahead_point.x}, y={lookahead_point.y}")
                dub = dubins(self.se_pose,lookahead_point,3,0.3,0.2,0.2)
                self.duckie_path = dub.solve()
                if self.check_collision([self.corner.shapely_obs],self.duckie_path):
                    rospy.loginfo("Collision detected, recalculating path")
                    dub1 =  dubins(self.se_pose,self.corner.placement,3,0.3,0.2,self.corner.radius)
                    self.duckie_path = dub1.solve()
                    dub2 = dubins(self.corner.placement,lookahead_point,3,0.3,self.corner.radius,0.2)
                    self.duckie_path += dub2.solve()
                self.publish_path_markers()

                temp_path_array= np.empty((0,3))
                for segment in self.duckie_path:
                    partial = segment.get_path_array()
                    temp_path_array = np.vstack((temp_path_array, partial))
                self.pursuit_path = temp_path_array
                self.running_dubs = True
            elif not self.do_dubins and not self.running_dubs:
                self.duckie_path =  [DuckieSegment(self.se_pose, lookahead_point, 0, 'STRAIGHT',sg.LineString([(self.se_pose.x,self.se_pose.y), (lookahead_point.x, lookahead_point.y)]),cost = 1)]
                self.publish_path_markers()
                temp_path_array= np.empty((0,3))
                for segment in self.duckie_path:
                    partial = segment.get_path_array()
                    temp_path_array = np.vstack((temp_path_array, partial))
                self.pursuit_path = temp_path_array

            l_speed, r_speed =self.pure_pursuit_control(self.pursuit_path, 0.04, 0.102, 0.1)
            wheels_cmd = WheelsCmdStamped()
            wheels_cmd.header.stamp = rospy.Time.now()
            wheels_cmd.vel_left = l_speed
            wheels_cmd.vel_right = r_speed
            self.wheel_cmd_pub.publish(wheels_cmd)
            # if self.running_dubs:
            #     rospy.loginfo(f"Left speed: {l_speed}, Right speed: {r_speed} doiiing the stuuf")
            # else:
            #     rospy.loginfo(f"Left speed: {l_speed}, Right speed: {r_speed} going straight")
            self.check_completion()

            # Publish markers for visualization
            self.publish_markers(lookahead_point)

            rate.sleep()


if __name__ == '__main__':
    node = DubinsNode()
    node.run()
#     rospy.spin()