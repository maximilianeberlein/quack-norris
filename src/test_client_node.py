import rospy
from geometry_msgs.msg import Pose2D

from quack_norris_utils.utils import SETransform, DuckieNode # type: ignore
from quack_norris_utils.utils import initialize_map, update_map # calculate_shortest_path # type: ignore
from quack_norris_utils.utils import TILE_DATA, print_path # type: ignore

TILE_SIZE = TILE_DATA["TILE_SIZE"]

if __name__ == "__main__":
    rospy.init_node("map_client_node")

    #################################################### Example Main Map ####################################################
    start_node = DuckieNode(
        pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
        tag_id=1 # Starting at index (0, 0)
    )
    end_node = DuckieNode(
        pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
        tag_id=6 # Ending at index (4, 3)
    )
    shortest_path = initialize_map(start_node, end_node)
    print_path(shortest_path)

    # Obstacle found! Update the map
    curr_pos = SETransform(x=TILE_SIZE*5.5, y=TILE_SIZE*2.5, theta=-1) # At index (5, 2)
    faulty_node = DuckieNode(
        pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
        tag_id=7 # Last known position node. In this case, exactly our current position
    )
    shortest_path_updated = update_map(curr_pos, faulty_node)
    print_path(shortest_path_updated)

    # Oh no! Another obstacle found! Update the map again
    curr_pos = SETransform(x=TILE_SIZE*2.2, y=TILE_SIZE*2.2, theta=-1) # Approximately at index (2, 2)
    faulty_node = DuckieNode(
        pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
        tag_id=5 # Last known position node. In this case, approximately our current position
    )
    shortest_path_updated_again = update_map(curr_pos, faulty_node)
    print_path(shortest_path_updated_again)

    ################################################### Example Circle Map ###################################################
    # start_node = DuckieNode(
    #     pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
    #     tag_id=1 # Starting at index (0, 0)
    # )
    # end_node = DuckieNode(
    #     pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
    #     tag_id=4 # Ending at index (2, 2)
    # )
    # shortest_path = initialize_map(start_node, end_node)
    # print_path(shortest_path)

    # # Obstacle found! Update the map
    # curr_pos = SETransform(x=TILE_SIZE*2.5, y=TILE_SIZE*1.5, theta=-1) # At index (2, 1)
    # faulty_node = DuckieNode(
    #     pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
    #     tag_id=2 # Last known position node. In this case, tile just "south of" our current position
    # )
    # shortest_path_updated = update_map(curr_pos, faulty_node)
    # print_path(shortest_path_updated)

    #################################################### Example Oval Map ####################################################
    # start_node = DuckieNode(
    #     pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
    #     tag_id=1 # Starting at index (0, 0)
    # )
    # end_node = DuckieNode(
    #     pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
    #     tag_id=4 # Ending at index (4, 2)
    # )
    # shortest_path = initialize_map(start_node, end_node)
    # print_path(shortest_path)

    # # Obstacle found! Update the map
    # curr_pos = SETransform(x=TILE_SIZE*2.5, y=TILE_SIZE*2.5, theta=-1) # At index (2, 2)
    # faulty_node = DuckieNode(
    #     pose=SETransform(x=-1, y=-1, theta=-1), # Position does not matter, ID does
    #     tag_id=2 # Last known position node. In this case, tile "south of" our current position
    # )
    # shortest_path_updated = update_map(curr_pos, faulty_node)
    # print_path(shortest_path_updated)