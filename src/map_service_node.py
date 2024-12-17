#!/usr/bin/env python3

import rospy
import csv
import numpy as np
import random

from geometry_msgs.msg import Pose2D

from quack_norris.srv import Map, MapResponse   # type: ignore
from quack_norris.msg import Node, Corner  # type: ignore

from quack_norris_utils.utils import find_closest_points, plot_solved_graph   # type: ignore
from quack_norris_utils.utils import fill_path_corners, MapNode   # type: ignore

from typing import List, Dict, Tuple, Optional

class MapServiceNode:
    def __init__(self):
        # Internal states to store the map
        self.nodes: List[MapNode]   = []
        self.goal_node:  MapNode    = None
        self.path:  List[MapNode]   = []  # Saves last saved path. Used to remove path with obstacles

        # Init
        self.csv_file   = rospy.get_param('~map_csvfile', '/code/catkin_ws/src/user_code/quack-norris/params/csv/main_map_test.csv')
        self.service    = rospy.Service("map_service", Map, self.handle_request)
        rospy.loginfo("Map service ready.")

    def load_map(self):
        # Load the map from the CSV file
        with open(self.csv_file, mode='r') as file:
            csv_reader = csv.reader(file)
            header = next(csv_reader)  # Skip the header
            for row in csv_reader:
                # Check if the node already exists
                if not self.get_index_via_id(int(row[2])) and not self.get_index_via_pos((int(row[0]), int(row[1]))):
                    self.nodes.append(MapNode(int(row[0]), int(row[1]), int(row[2])))
                elif self.get_index_via_id(int(row[2])) == self.get_index_via_pos((int(row[0]), int(row[1]))):
                    rospy.loginfo(f"Node with index ({int(row[0])}, {int(row[1])}) and apriltag_id {int(row[2])} exists more than once in map csv file - Ignoring...")
                else:
                    rospy.logerr(f"Two nodes with same position/apriltag_id found in map csv file, but resp. apriltag_id/position don't match. Exiting...")
                    exit(1)

        # Connect the nodes
        for node in self.nodes:
            neighbour_positions = find_closest_points(node.center_index, [n.center_index for n in self.nodes if n != node])
            for neighbour_position in neighbour_positions:
                neighbour_index = self.get_index_via_pos(neighbour_position)
                if neighbour_index is not None:
                    neighbour = self.nodes[neighbour_index]
                    node.neighbors.append(neighbour)

    def get_index_via_pos(self, pos: Tuple[int, int]) -> Tuple[int, int]:
        for index, node in enumerate(self.nodes):
            if node.center_index == pos:
                return index
        return None
    
    def get_index_via_id(self, apriltag_id: int) -> Tuple[int, int]:
        for index, node in enumerate(self.nodes):
            if node.apriltag_id == apriltag_id:
                return index
        return None
    
    def get_node(self, apriltag_id: int = None, center_index: Tuple[int, int] = None) -> MapNode:
        # Apriltag has priority
        if apriltag_id is not None:
            index = self.get_index_via_id(apriltag_id)
            if index is None:
                rospy.logerr(f"Node with apriltag ID {apriltag_id} not found. Exiting...")
                exit(1)
        elif center_index is not None:
            index = self.get_index_via_pos(center_index)
            if index is None:
                rospy.logerr(f"Node with center {center_index} not found. Exiting...")
                exit(1)
        else:
            rospy.logerr("No apriltag ID or center index provided when trying to find node - Should not happen. Exiting...")
            exit(1)

        return self.nodes[index]

    def handle_request(self, req):
        if req.reset:
            # Reset
            self.nodes = []
            self.path = []
            self.load_map()

            start_node  = self.get_node(req.start_node.apriltag_id)
            goal_node   = self.get_node(req.goal_node.apriltag_id)
            self.goal_node = goal_node
        else:
            # Update - remove faulty edge to planned next node
            start_node  = self.get_node(req.start_node.apriltag_id)
            goal_node   = self.goal_node

            # Find planned next node in path
            if start_node not in self.path:
                rospy.logerr(f"While updating map: Start node {start_node} not in original path. Exiting...")
                exit(1)
            for index, node in enumerate(self.path):
                if node == start_node:
                    break
            next_node = self.path[index + 1]

            # Remove planned next node from current node's neighbors, as it is blocked
            for index, node in enumerate(start_node.neighbors):
                if node.center_index == next_node.center_index:
                    start_node.neighbors.pop(index)
                    break

        shortest_path = self.astar_search(start_node, goal_node)

        if shortest_path is not None:
            self.path = shortest_path
            # plot_solved_graph(self.nodes, shortest_path, f"/home/duckie/repos/quack-norris/src/solved_graph{random.randint(0, 1000)}.png")
            full_path = fill_path_corners(shortest_path)
            return MapResponse(
                success=True,
                message=f"Successfully calculated shortest path from ({req.start_node.pose.x}, {req.start_node.pose.y}) to ({req.goal_node.pose.x}, {req.goal_node.pose.y})",
                nodes=full_path
            )
        
        return MapResponse(
            success=False,
            message="No path found",
            nodes=[]
        )

    def astar_search(self, start: MapNode, goal: MapNode) -> List[MapNode]:
        """
        A* search algorithm to find the optimal path from start to goal.

        Args:
            start (MapNode): Start node
            goal (MapNode): Goal node

        Returns:
            List[MapNode]: Path from start to goal if found; otherwise, an empty list.
        """
        def heuristic(node1: MapNode, node2: MapNode) -> float:
            """
            Heuristic function for A* search. Describes the weight between two nodes (includes the heuristic to the goal node).
            """
            return np.linalg.norm(np.array(node1.center_index) - np.array(node2.center_index))
        
        open_set:       Dict[MapNode, float]                = {start: 0}    # {Nodes to explore: Best cost-to-reach}
        cost_to_reach:  Dict[MapNode, float]                = {start: 0}    # {Visited Nodes: Best cost-to-reach}
        Parents:        Dict[MapNode, Optional[MapNode]]    = {start: None} # {Visited Nodes: Best Parent}

        while open_set:
            # First check node with minimum total cost (current_cost + heuristic)
            current_node = min(open_set, key=open_set.get)
            open_set.pop(current_node)

            if current_node == goal:
                # Path found. Backtrack to construct the path
                path = [current_node]
                while Parents[current_node] is not None:
                    current_node = Parents[current_node]
                    path.insert(0, current_node)
                return path

            for neighbor in current_node.neighbors:
                edge_cost = heuristic(current_node, neighbor)
                new_cost_to_reach = cost_to_reach[current_node] + edge_cost

                # Neighbor is new or found a cheaper path
                if neighbor not in cost_to_reach or new_cost_to_reach < cost_to_reach[neighbor]:
                    cost_to_reach[neighbor] = new_cost_to_reach
                    total_estimated_cost = new_cost_to_reach + heuristic(neighbor, goal)
                    open_set[neighbor] = total_estimated_cost
                    Parents[neighbor] = current_node

        # No path was found
        return None

if __name__ == "__main__":
    rospy.init_node("map_service_node")
    MapServiceNode()
    rospy.spin()