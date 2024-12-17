import rospy
from quack_norris.srv import Map    # type: ignore
from quack_norris.msg import Node, Corner   # type: ignore
from geometry_msgs.msg import Pose2D

from quack_norris_utils.utils import SETransform, DuckieNode # type: ignore
from quack_norris_utils.utils import initialize_map, update_map # calculate_shortest_path # type: ignore
from quack_norris_utils.utils import print_path # type: ignore

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
    # shortest_path = calculate_shortest_path(start_node, end_node)
    shortest_path = initialize_map(start_node, end_node)
    print_path(shortest_path)

    # Obstacle found! Update the map
    # curr_pos = DuckieNode(SETransform(x=3.2175, y=1.4625, theta=-1))
    curr_pos = SETransform(x=3.2175, y=1.4625, theta=-1)
    faulty_node = DuckieNode(
        pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
        tag_id=7
    )
    # shortest_path_updated = calculate_shortest_path(start_node, end_node)
    shortest_path_updated = update_map(curr_pos, faulty_node)
    print_path(shortest_path_updated)

    # # Oh no! Another obstacle found! Update the map again
    # # curr_pos = DuckieNode(SETransform(x=1.4, y=1.4, theta=-1))
    # curr_pos = SETransform(x=1.4, y=1.4, theta=-1)
    # faulty_node = DuckieNode(
    #     pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
    #     tag_id=5
    # )
    # # end_node.tag_id = 100
    # # shortest_path_updated_again = calculate_shortest_path(start_node, end_node)
    # shortest_path_updated_again = update_map(curr_pos, faulty_node)
    # print_path(shortest_path_updated_again)
    