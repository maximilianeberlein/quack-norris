#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Point
from tf.transformations import quaternion_from_euler
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped, BoolStamped
from std_msgs.msg import Float32, ColorRGBA
from quack_norris.msg import TagInfo
import shapely.geometry as sg
import matplotlib.pyplot as plt
import yaml
import tf
from typing import List
import os
import numpy as np

from quack_norris_utils.utils import dubins, SETransform, DuckieObstacle, DuckieNode, DuckieSegment, DuckieCorner, DuckieDriverObstacle
from visualization_msgs.msg import Marker

from dynamic_reconfigure.server import Server
from quack_norris.cfg import DubinsNodeConfig


##### Define the hardcoded path for testing on the medium map #####

a = 0.585/4
p1 = SETransform(a, a, 3*np.pi/2)
p2 = SETransform(19*a, a, 0)
p3 = SETransform(19*a, 11*a, np.pi/2)
p4 = SETransform(a, 11*a, np.pi)

p1 = DuckieNode(p1, tag_id=395)
p2 = DuckieNode(p2, tag_id=74)
p3 = DuckieNode(p3, tag_id=392)
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
p3.insert_corner(DuckieCorner(SETransform(16*a,8*a, 3*np.pi/4),3*a,'LEFT'))
p4.insert_corner(DuckieCorner(SETransform(4*a,8*a, 5*np.pi/4),3*a,'LEFT'))
hardcoded_path = [p2, p3, p4, p1]



class DubinsNode:
    def __init__(self):
        rospy.init_node('dubins_node')

        self.bot_name = os.environ.get("VEHICLE_NAME", "duckiebot")
        self.yaml_file = rospy.get_param('~yaml_file', '/code/catkin_ws/src/user_code/quack-norris/params/apriltags.yaml')

        self.wheel_base = rospy.get_param('~wheel_base', 0.102)

        ##### load obstacle information from yaml file #####
        self.obstacles = self.load_obstacle(self.yaml_file)
        self.obstacle_dict = {obstacle['id']: obstacle for obstacle in self.obstacles}

        ##### here the pose is initialized to (0,0,0) #####
        self.se_pose = SETransform(0,0,0)
        ########## Subscriber for odometry and tag info ##########
        self.odom_sub = rospy.Subscriber('/wheel_encoder/odom', Odometry, self.odom_callback)
        self.tag_info_sub = rospy.Subscriber(f'/{self.bot_name}/tag_info', TagInfo, self.tag_info_callback, buff_size = 1)
        ########## Publisher for rviz  ##########
        self.marker_pub = rospy.Publisher('/dubins_marker', Marker, queue_size=1)
        self.dubins_pub = rospy.Publisher('/dubins_path', Marker, queue_size=1)
        ########## Publisher for wheel commands and angular speed, goes to wheel speed controller ##########
        self.desired_wheel_cmd_pub = rospy.Publisher(f'/{self.bot_name}/desired_wheel_speed', WheelsCmdStamped, queue_size=1)
        self.desired_angular_speed = rospy.Publisher(f'/{self.bot_name}/desired_angular_speed', Float32, queue_size=1)
        
        #####global path for the duckiebot to follow #####
        self.path = hardcoded_path
        ##### base agressiveness for the corners #####
        self.base_corner_radius = 3*a

        # Set up dynamic reconfigure
        self.dyn_reconf_server = Server(DubinsNodeConfig, self.dyn_reconf_callback)


        ### variable for checking  path following and activating dubins ####
        self.tag_info = None
        self.tag_distance = 10
        self.node_lookahead = 6*a
        self.waypoint_lookahead = 10.0*a
        self.next_node = None
        self.node_in_scope = False
        self.tag_present = False

        ##### variables for dubins #####
        self.line = None
        self.lookahead_point =None
        self.line_theta = None
        self.duckie_path = None
        self.pursuit_path = None
        self.running_dubs = False
        self.do_dubins = False
        self.corner = None
        ##### speed variables  scale the wheel command by v_max#####
        self.speed = 0.4
        self.v_max = 0.8
        
        ##### obstacle variables #####
        self.obstacle_ids =[390,391]
        self.obstacle= None
        self.obstacle_avoid = False

        rospy.on_shutdown(self.shutdown_duckie)

    def load_obstacle(self, yaml_file):
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
        return data['obstacle_tags']

    ###### dynamic reconfigure callback for changing the aggressiveness of the corners #####
    def dyn_reconf_callback(self, config, level):
        # config.aggressiveness is an int from 0 to 3
        self.aggressiveness = config.aggressiveness
        rospy.loginfo(f"Dynamic Reconfigure: Aggressiveness set to {self.aggressiveness}")
        new_radius = self.base_corner_radius - self.aggressiveness * a/2 
        # Update the corners based on the new aggressiveness
        for p in self.path:
            p.corner.update_corners(new_radius)
        return config
    

    def odom_callback(self, msg):
        ##### get the pose of the duckiebot #####
        self.se_pose.x = msg.pose.pose.position.x
        self.se_pose.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.se_pose.theta = yaw
    
    def tag_info_callback(self, msg):
        ####### get the tag info regarding distance from the duckie ######
        self.tag_info = msg
        self.tag_distance = np.sqrt(self.tag_info.x**2 + self.tag_info.y**2)


    def get_node_lookahead(self):
        ##### check if the next node is in scope of the duckiebot #####
        if sg.Point(self.se_pose.x, self.se_pose.y).buffer(self.node_lookahead).contains(sg.Point(self.next_node.x, self.next_node.y)):
            # rospy.loginfo(f"Node at position{self.next_node.x,self.next_node.y} is in scope")
            self.node_in_scope = True
        else:
            self.node_in_scope = False 

    def check_tag(self):
        ###### check if the tag is present and if it is the correct tag for the associated node ######
        if self.tag_info is not None:
            rospy.loginfo(f"Tag {self.tag_info.tag_id} detected")
            if self.next_node.tag_id == self.tag_info.tag_id and self.tag_distance < 9*a:
                #rospy.loginfo(f"Tag {self.tag_info.tag_id} is present, we moving on brahh")
                self.tag_present = True
            ##### if the tag is not the correct tag for the node, check if it is an obstacle ######
            elif self.tag_info.tag_id in self.obstacle_dict and not self.obstacle_avoid :
                matching_obstacle = self.obstacle_dict[self.tag_info.tag_id]
                speed = matching_obstacle['speed']
                pose = SETransform(self.se_pose.x + self.tag_info.x, self.se_pose.y + self.tag_info.y, self.line_theta)
                self.obstacle = DuckieDriverObstacle(pose,speed,self.waypoint_lookahead/self.speed, self.lookahead_point)
                self.tag_present = False
                        
            else:
                self.tag_present = False
            if self.obstacle_avoid:
                self.tag_present = False
        else:
            self.tag_present = False
    
    def check_collision(self,obstacles: List[sg.Polygon], duckie_path : List[DuckieSegment]):
        #####check if the base dubins case is collision free #####
        #### set the flag to replan if necessary ####
        collision = False
        for segment in duckie_path:
            for obstacle in obstacles:
                if segment.shapely_path.intersects(obstacle):
                    collision = True
                    break
        return collision
    
    
    def get_line(self):
        ##### get the line and its heading from current node and next node #####
        ##### the duckie will intersect this line to get the lookahead point #####
        self.line = sg.LineString([(self.next_node.parent.x,self.next_node.parent.y), (self.next_node.x, self.next_node.y)])
        self.line = self.extend_line(self.line, 0.5)
        self.line_theta = np.arctan2(self.line.coords[1][1] - self.line.coords[0][1], self.line.coords[1][0] - self.line.coords[0][0])
    

    def get_line_lookahead(self):
        ##### get the lookahead point on the line #####
        circle = sg.Point(self.se_pose.x, self.se_pose.y).buffer(self.waypoint_lookahead).boundary
        intersection = circle.intersection(self.line)
        # rospy.loginfo(f"Intersection: {intersection}")

        if intersection.is_empty:
            return SETransform(self.next_node.x, self.next_node.y, self.line_theta)
        elif intersection.geom_type == 'Point':
            intersect = SETransform(intersection.x, intersection.y, self.line_theta)
            return self.decrease_lookahead(intersect,-0.2)
        ##### if the lookahead intersect at multiple points of the line #####
        # Convert MultiPoint to a list of Points
        elif intersection.geom_type == 'MultiPoint':
        # Find the closest point to the waypoint
            closest_point = min(list(intersection.geoms), key=lambda point: point.distance(sg.Point(self.next_node.x, self.next_node.y)))
            return SETransform(closest_point.x, closest_point.y, self.line_theta)
        else:
            rospy.logwarn(f"Unexpected intersection type: {intersection.geom_type}")
            return SETransform(self.next_node.x, self.next_node.y, self.line_theta)
        
    def publish_markers(self, lookahead_point):
        #### if you desire visualisation of the lookahead point and the line ####
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
        #### if you desire visualisation of the lookahead point and the line ####
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
        
        # Publish the last corner radius

        if self.corner:
            corner_marker = Marker()
            corner_marker.header.frame_id = "map"
            corner_marker.header.stamp = rospy.Time.now()
            corner_marker.ns = "dubins_path"
            corner_marker.id = segment_id
            corner_marker.type = Marker.SPHERE
            corner_marker.action = Marker.ADD
            corner_marker.scale.x = self.corner.radius * 2
            corner_marker.scale.y = self.corner.radius * 2
            corner_marker.scale.z = 0.01  # Flat sphere
            corner_marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.5)  # Red color, semi-transparent
            corner_marker.pose.position.x = self.corner.pose.x
            corner_marker.pose.position.y = self.corner.pose.y
            corner_marker.pose.position.z = 0.0
            self.marker_pub.publish(corner_marker)
    
    def pure_pursuit_control(self, path, lookahead_distance, wheelbase, speed):
        # If the path is empty, return 0,0 
        if self.pursuit_path is None:
            return 0,0
        x_path = self.pursuit_path[:, 0]
        y_path = self.pursuit_path[:, 1]
        theta_path = self.pursuit_path[:, 2]
        
        # Find the closest point on the path
    
        distances = np.sqrt((x_path - self.se_pose.x)**2 + (y_path - self.se_pose.y)**2)
        closest_index = np.argmin(distances)

        if distances[-1] < lookahead_distance:
            closest_index = len(distances) -1 

        # Find the lookahead point
        lookahead_index = closest_index
        

        
        while lookahead_index < len(path) and distances[lookahead_index] < lookahead_distance:
            lookahead_index += 1

        if lookahead_index >= len(path):
            lookahead_index = len(path) - 1

            

        lookahead_point = path[lookahead_index]
        # print(f'closest point{lookahead_index}, x {lookahead_point}')
        self.pursuit_path = self.pursuit_path[lookahead_index:]
        
        # Calculate the steering angle
        alpha = np.arctan2(lookahead_point[1] - self.se_pose.y, lookahead_point[0] - self.se_pose.x) - self.se_pose.theta
        steering_angle = np.arctan2(2 * wheelbase * np.sin(alpha), lookahead_distance)

        desired_angular_speed = lookahead_point[3] 
        self.desired_angular_speed.publish(Float32(desired_angular_speed))
        # Calculate the left and right wheel speeds
        l_speed = (speed - (steering_angle * wheelbase/2)) 
        r_speed = (speed + (steering_angle * wheelbase/2))
        # rospy.loginfo(f'l {l_speed},r {r_speed} ')

        return l_speed, r_speed
    def normalize_angle(self,angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def complete_heading_correction(self,target_angle: float):
        ##### this function can  help to stabilize the duckie if it has large heading deviations,
        ##### but is ultimately not smooth######
        ##### complete the heading correction to the target angle #####
        fixed_speed = 0.4
        omega =  fixed_speed/(self.wheel_base/2)
        angle_deviation = self.normalize_angle(target_angle) - self.normalize_angle(self.se_pose.theta)
        time = 0.05
        rate = rospy.Rate(20)
        while angle_deviation > 0.05:
            l_speed = -fixed_speed*np.sign(angle_deviation)
            r_speed = fixed_speed*np.sign(angle_deviation)
            wheels_cmd = WheelsCmdStamped()
            wheels_cmd.header.stamp = rospy.Time.now()
            wheels_cmd.vel_left = l_speed
            wheels_cmd.vel_right = r_speed
            self.desired_angular_speed.publish(0.0)
            self.desired_wheel_cmd_pub.publish(wheels_cmd)
            rospy.sleep(time)
            wheels_cmd = WheelsCmdStamped()
            wheels_cmd.header.stamp = rospy.Time.now()
            wheels_cmd.vel_left = 0
            wheels_cmd.vel_right = 0
            self.desired_wheel_cmd_pub.publish(wheels_cmd)
            angle_deviation = self.normalize_angle(target_angle) - self.normalize_angle(self.se_pose.theta)
            rate.sleep()

    def u_turn(self,target_angle: float):
        #####  we use this function to avoid obstacles, it is a hard turn  and a global map call is needed after calling u_turn#####
        fixed_speed = 0.4
        omega =  fixed_speed/(self.wheel_base/2)
        rate = rospy.Rate(20)
        angle_deviation = self.normalize_angle(target_angle) - self.normalize_angle(self.se_pose.theta)
        time = 0.05
        while angle_deviation > 0.05:
            l_speed = 0#-fixed_speed*np.sign(angle_deviation)
            r_speed = fixed_speed#*np.sign(angle_deviation)
            wheels_cmd = WheelsCmdStamped()
            wheels_cmd.header.stamp = rospy.Time.now()
            wheels_cmd.vel_left = l_speed
            wheels_cmd.vel_right = r_speed
            self.desired_angular_speed.publish(0.0)
            self.desired_wheel_cmd_pub.publish(wheels_cmd)
            rospy.sleep(time)
            wheels_cmd = WheelsCmdStamped()
            wheels_cmd.header.stamp = rospy.Time.now()
            wheels_cmd.vel_left = 0
            wheels_cmd.vel_right = 0
            self.desired_wheel_cmd_pub.publish(wheels_cmd)
            angle_deviation = self.normalize_angle(target_angle) - self.normalize_angle(self.se_pose.theta)
            rate.sleep()
        self.tag_info = None
        self.shutdown_duckie()


    def shutdown_duckie(self):
        rospy.loginfo("Shutting down... stopping the robot.")
        self.desired_wheel_cmd_pub.publish(WheelsCmdStamped())
        rospy.sleep(1)    

    def get_temp_line(self, pose, point):
        ##### use this function to get a temporary line for extending until the node#####
        #### or if not using dubins, to get a line to the lookahead point #####
        x_coords = np.linspace(pose.x, point.x, 30)
        y_coords = np.linspace(pose.y, point.y, 30)
        
        # Combine x and y coordinates into a list of tuples
        points = list(zip(x_coords, y_coords))
        
        # Create a LineString from the points
        return sg.LineString(points)
    
    def run(self):
        rate = rospy.Rate(20)
        #init by popping the first node 
        self.next_node = self.path[0]
        rospy.wait_for_message('/wheel_encoder/odom', Odometry)
        while not rospy.is_shutdown():
            
            # Check if the next node is in scope 
            self.check_tag()
        
            # If the next_node is in scope and the tag matches, move on to the next node
            if self.tag_present :
                # initialize the corner and the next node
                rospy.loginfo(f"Moving to next node")
                self.corner = self.next_node.corner
                self.next_node = self.next_node.next
                if self.path == []:
                    self.path = hardcoded_path
                # set the necessary flags
                self.do_dubins = True
                self.running_dubs = False
                self.node_in_scope = False
                self.tag_present = False
                self.tag_info = None
                self.get_line()  # Update line for new node
            

            # If we have a next_node but no line yet, compute it, this only gets used at the start
            if self.next_node and self.line is None:
                self.get_line()

            # get the lookahead point on the line
            self.lookahead_point = self.get_line_lookahead()
            lookahead_point = self.lookahead_point

            # check always first if an obstacle is present
            if self.obstacle != None  and not self.obstacle_avoid:
                rospy.loginfo(f"SUUUUPER DANGER, lets dash and burn rubber")
                self.u_turn(self.se_pose.theta + np.pi)
                # replan here


            
            # check if there is a valid lookahead point, if we need to plan a dubins path and not already running a dubins  
            elif lookahead_point and self.do_dubins and not self.running_dubs:
                goal_pose =  lookahead_point
                #quick stop for debbugging and to see when the dubins is called
                self.desired_wheel_cmd_pub.publish(WheelsCmdStamped())
                #first check if the base dubins is collision free
                dub = dubins(self.se_pose,goal_pose,3,self.speed,0.2,0.2)
                self.duckie_path = dub.solve()
                if self.check_collision([self.corner.shapely_obs],self.duckie_path):
                    ### if collision, recalculate the path with the corner radius
                    rospy.loginfo(f"Collision detected, recalculating path,goal pose{goal_pose.x,goal_pose.y}")
                    
                    dub1 =  dubins(self.se_pose,self.corner.placement,3,self.speed,0.2,self.corner.radius)
                    path1 = dub1.solve()
                    dub2 = dubins(self.corner.placement,goal_pose,3,self.speed,self.corner.radius,0.2)
                    path2= dub2.solve()
                    # for stability reasons we add a straight line to the next node here, so that the pure pursuit has enough waypoints
                    temp_line = self.get_temp_line(goal_pose, self.next_node.pose)
                    temp_path =  [DuckieSegment(goal_pose, self.next_node.pose, 0, 'STRAIGHT',temp_line,cost = 1,speed = self.speed)]
                    self.duckie_path = np.concatenate((path1,path2, temp_path)) 
                    print(self.duckie_path)
                # self.publish_path_markers() #### debug only

                #### stitch the path together and get the path array for the pure pursuit we will get [x,y,theta,angular speed, radius] ####
                temp_path_array= np.empty((0,5))
                for segment in self.duckie_path:
                    partial = segment.get_path_array()
                    temp_path_array = np.vstack((temp_path_array, partial))
                self.pursuit_path = temp_path_array
                # set the flag to avoid arbitray path and further dubins calls
                self.running_dubs = True
                self.do_dubins = False
            
            ##### if we are not running dubins, we are only following the lane #####
            elif not self.do_dubins and not self.running_dubs and not self.obstacle_avoid:
                rospy.loginfo(f'only lane following')
                #get the arbitrary path to the lookahead point
                temp_line = self.get_temp_line(self.se_pose, lookahead_point)
                self.duckie_path =  [DuckieSegment(self.se_pose, lookahead_point, 0, 'STRAIGHT',temp_line,cost = 1,speed = self.speed)]
                self.publish_path_markers()
                temp_path_array= np.empty((0,5))
                for segment in self.duckie_path:
                    rospy.logwarn(f"Segment end: ({segment.end.x},{segment.end.y})")
                    partial = segment.get_path_array()
                    temp_path_array = np.vstack((temp_path_array, partial))
                rospy.loginfo(f"Path array: {temp_path_array}")
                self.pursuit_path = temp_path_array
                # possible different pure pursuit parameters for the lane following
                l_speed, r_speed =self.pure_pursuit_control(self.pursuit_path, 0.12, 0.102, self.speed)
                

            if self.running_dubs:
                # possible different pure pursuit parameters for the dubins
                l_speed, r_speed =self.pure_pursuit_control(self.pursuit_path, 0.12, 0.102, self.speed)

            # Publish the wheel commands
            wheels_cmd = WheelsCmdStamped()
            wheels_cmd.header.stamp = rospy.Time.now()
            wheels_cmd.vel_left = l_speed / self.v_max
            wheels_cmd.vel_right = r_speed / self.v_max
            self.desired_wheel_cmd_pub.publish(wheels_cmd)
            
            # Publish markers for visualization if debug
            # self.publish_markers(lookahead_point)

            rate.sleep()


if __name__ == '__main__':
    node = DubinsNode()
    node.run()
