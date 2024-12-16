import rospy
import csv
import numpy as np
import random

from geometry_msgs.msg import Pose2D

from quack_norris.srv import Map, MapResponse   # type: ignore
from quack_norris.msg import Node, Corner  # type: ignore

# from quack_norris_utils.utils import corner_to_duckiecorner, node_to_duckienode   # type: ignore
from quack_norris_utils.utils import find_closest_points, plot_solved_graph   # type: ignore
# from quack_norris_utils.utils import TILE_SIZE  # type: ignore
TILE_DATA = {"TILE_SIZE": 0.585, "D_CENTER_TO_CENTERLINE": 0.2925, "CENTERLINE_WIDTH": 0.025, "LANE_WIDTH": 0.205} # m

from typing import List, Dict, Tuple, Optional

class MapNode:
    def __init__(self, index_x: int, index_y: int, apriltag_id: int):
        self.center_index:  Tuple[int, int]     = (index_x, index_y)
        self.apriltag_id:   int                 = apriltag_id
        self.neighbors:     List[MapNode]       = []

    def __repr__(self):
        return f"MapNode(Center: {self.center_index}, ID: {self.apriltag_id})"
    
def node_to_mapnode(node: Node) -> MapNode:
    index_x, index_y = tile_pos_to_index((node.pose.x, node.pose.y))
    return MapNode(index_x, index_y, node.apriltag_id)

def mapnode_to_node(mapnode: MapNode, corner: Corner) -> Node:
    xabs, yabs = tile_index_to_pos(mapnode.center_index)
    return Node(pose=Pose2D(x=xabs, y=yabs, theta=0), # Theta?
                apriltag_id=mapnode.apriltag_id,
                corner=corner)

def tile_pos_to_index(pos: Tuple[float, float]) -> Tuple[int, int]:
    rospy.loginfo(f"Converting position {pos} to index {int(pos[0] / TILE_DATA['TILE_SIZE'])}, {int(pos[1] / TILE_DATA['TILE_SIZE'])}")
    return (int(pos[0] / TILE_DATA["TILE_SIZE"]), int(pos[1] / TILE_DATA["TILE_SIZE"]))

def tile_index_to_pos(index: Tuple[int, int]) -> Tuple[float, float]:
    return ((index[0] + 0.5) * TILE_DATA["TILE_SIZE"], (index[1] + 0.5) * TILE_DATA["TILE_SIZE"])

class MapServiceNode:
    def __init__(self):
        # Map initialization or update
        self.initialized = False

        # Internal state to store the map
        self.nodes: List[MapNode]   = []
        self.path:  List[MapNode]   = []  # Saves last saved path. Used to remove path with obstacles
        self.csv_file = rospy.get_param('~map_csvfile', '/code/catkin_ws/src/user_code/quack-norris/params/csv/main_map.csv')
        self.load_map()

        # Create the service
        self.service = rospy.Service("map_service", Map, self.handle_request)
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

        # raw_path = self.astar_search(self.nodes[0], self.nodes[5])
        # path = self.fill_path_corners(raw_path)
        # plot_solved_graph(self.nodes, raw_path, "/home/duckie/repos/quack-norris/src/solved_graph.png")
        # exit(1)

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
    
    def get_node(self, index: int) -> MapNode:
        if index is None:
            rospy.logerr(f"Node index is None (i.e. Node may not exist). Exiting...")
            exit(1)
        if index < 0 or index >= len(self.nodes):
            rospy.logerr(f"Node index {index} out of bounds. Exiting...")
            exit(1)
        return self.nodes[index]
    
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
        # rospy.loginfo(f"Received request: {req}")
        # start_node  = self.nodes[self.get_index_via_id(req.start_node.apriltag_id)]
        # goal_node   = self.nodes[self.get_index_via_id(req.goal_node.apriltag_id)]
        start_node  = self.get_node(req.start_node.apriltag_id)
        goal_node   = self.get_node(req.goal_node.apriltag_id)
        if self.initialized: # Remove path with obstacle
            # rospy.loginfo(f"Removing path with obstacle from start node {start_node.center_index} to goal node {goal_node.center_index}")
            # Find planned next node in path
            if start_node not in self.path:
                rospy.logerr(f"While updating map: Start node {start_node} not in original path. Exiting...")
                exit(1)
            for index, node in enumerate(self.path):
                if node == start_node:
                    break
            # rospy.loginfo(f"Index: {index}. Length: {len(self.path)}")
            next_node = self.path[index + 1]

            # Remove planned next node from current node's neighbors, as it is blocked
            # rospy.loginfo(f"Removing node {next_node.center_index} from node {start_node.center_index}'s neighbors")
            # rospy.loginfo(f"Neighbors before: {start_node.neighbors}")
            for index, node in enumerate(start_node.neighbors):
                if node.center_index == next_node.center_index:
                    start_node.neighbors.pop(index)
                    break
            # rospy.loginfo(f"Neighbors after: {start_node.neighbors}")

        self.initialized = True
        shortest_path = self.astar_search(start_node, goal_node)

        # rospy.loginfo(f"Start: {start_node.center_index}, Goal: {goal_node.center_index}")
        if shortest_path is not None:
            self.path = shortest_path
            # plot_solved_graph(self.nodes, shortest_path, f"/home/duckie/repos/quack-norris/src/solved_graph{random.randint(0, 1000)}.png")
            full_path = self.fill_path_corners(shortest_path)
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
    
    def fill_path_corners(self, path: List[MapNode]) -> List[MapNode]:
        """
        Fill in the corners of each node in the path. Used in the local planner later.

        Args:
            path (List[MapNode]): Path to fill with corners

        Returns:
            List[MapNode]: Path with corners
        """
        filled_path = [mapnode_to_node(path[0], Corner())]
        for node in path[1:-1]:
            prev_node = path[path.index(node) - 1]
            next_node = path[path.index(node) + 1]

            # Find the corner between the previous and next node
            prev_node_pos = np.array(prev_node.center_index)
            next_node_pos = np.array(next_node.center_index)
            curr_node_pos = np.array(node.center_index)
            if np.array_equal(prev_node_pos, curr_node_pos) or np.array_equal(curr_node_pos, next_node_pos):
                raise ValueError("Two nodes in the path have the same position - Should not happen. Exiting...")
            vec_to = (curr_node_pos - prev_node_pos) / np.linalg.norm(curr_node_pos - prev_node_pos)
            vec_from = (next_node_pos - curr_node_pos) / np.linalg.norm(next_node_pos - curr_node_pos)
            dir = int(np.cross(vec_from, vec_to))
            
            corner_pos = np.array(tile_index_to_pos(curr_node_pos)) + (vec_from - vec_to) * TILE_DATA["TILE_SIZE"] / 2
            corner_theta = -dir * np.pi / 4 # +/-?
            corner_radius = abs(dir)*(TILE_DATA["D_CENTER_TO_CENTERLINE"] + TILE_DATA["CENTERLINE_WIDTH"] / 2 + TILE_DATA["LANE_WIDTH"] / 2)
            corner_type = dir # -1: LEFT, 0: STRAIGHT, 1: RIGHT
            corner = Corner(pose=Pose2D(x=corner_pos[0], y=corner_pos[1], theta=corner_theta),
                            radius=corner_radius,
                            type=corner_type)
            filled_path.append(mapnode_to_node(node, corner))
            
        filled_path.append(mapnode_to_node(path[-1], Corner()))
        return filled_path

if __name__ == "__main__":
    rospy.init_node("map_service_node")
    MapServiceNode()
    rospy.spin()