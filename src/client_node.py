import rospy
from quack_norris.srv import Map    # type: ignore
from quack_norris.msg import Node, Corner   # type: ignore
from geometry_msgs.msg import Pose2D

from quack_norris_utils.utils import SETransform, DuckieCorner, DuckieNode # type: ignore
from quack_norris_utils.utils import initialize_map, update_map # type: ignore

if __name__ == "__main__":
    rospy.init_node("map_client_node")

    # Initialize the map
    initialize_map()

    # Add a new node
    # rospy.loginfo("\n\n\nAdding a node to the map...")
    new_node = DuckieNode(
        pose=SETransform(x=1.0, y=2.0, theta=0.5),
        tag_id=123
    )
    new_node.insert_corner(DuckieCorner(pose=SETransform(x=0.5, y=0.5, theta=0.0), radius=0.0, type=0))
    update_map(node=new_node)

    # Add another node
    # rospy.loginfo("Adding another node to the map...")
    another_node = DuckieNode(
        pose=SETransform(x=2.0, y=3.0, theta=1.0),
        tag_id=456
    )
    another_node.insert_corner(DuckieCorner(pose=SETransform(x=1.0, y=1.5, theta=0.1), radius=1.0, type=-1))
    update_map(node=another_node)