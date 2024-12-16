import rospy
from quack_norris.srv import Map    # type: ignore
from quack_norris.msg import Node, Corner   # type: ignore
from geometry_msgs.msg import Pose2D

from quack_norris_utils.utils import SETransform, DuckieCorner, DuckieNode # type: ignore
from quack_norris_utils.utils import calculate_shortest_path # type: ignore

def print_path(path):
    path_str = "Calculated shortest path:\n"
    for index, node in enumerate(path):
        path_str += f"{node.tag_id}: ({node.pose.x}, {node.pose.y})"
        if index != len(path) - 1:
            path_str += f" -> "
    rospy.loginfo(path_str)

if __name__ == "__main__":
    rospy.init_node("map_client_node")
    start_node = DuckieNode(
        pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
        tag_id=1
    )
    end_node = DuckieNode(
        pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
        tag_id=6
    )
    shortest_path = calculate_shortest_path(start_node, end_node)
    path_str = ""
    print_path(shortest_path)

    # Obstacle found! Update the map
    start_node = DuckieNode(
        pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
        tag_id=7
    )
    shortest_path_updated = calculate_shortest_path(start_node, end_node)
    print_path(shortest_path_updated)

    # Oh no! Another obstacle found! Update the map again
    start_node = DuckieNode(
        pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
        tag_id=5
    )
    # end_node.tag_id = 100
    shortest_path_updated_again = calculate_shortest_path(start_node, end_node)
    print_path(shortest_path_updated_again)
    