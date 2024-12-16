import rospy
from test_service.srv import Map, MapResponse
from test_service.msg import Node, Corner

class MapServiceNode:
    def __init__(self):
        # Internal state to store the map
        self.nodes = []  # List of Node objects
        self.initialized = False  # Tracks if the map is initialized

        # Create the service
        self.service = rospy.Service("map_service", Map, self.handle_request)
        rospy.loginfo("Map service ready.")

    def handle_request(self, req):
        if req.start:  # Initialize the map
            if self.initialized:
                return MapResponse(
                    success=False,
                    message="Map is already initialized.",
                    nodes=self.nodes
                )

            self.nodes = []  # Clear the map
            self.initialized = True
            return MapResponse(
                success=True,
                message="Map initialized successfully.",
                nodes=self.nodes
            )

        else:  # Update the map with the new Node
            if not self.initialized:
                return MapResponse(
                    success=False,
                    message="Map is not initialized. Call with 'start=True' first.",
                    nodes=[]
                )

            # Add the new Node to the map
            self.nodes.append(req.node)
            return MapResponse(
                success=True,
                message=f"Node with AprilTag ID {req.node.apriltag_id} added to the map.",
                nodes=self.nodes
            )

if __name__ == "__main__":
    rospy.init_node("map_service_node")
    MapServiceNode()
    rospy.spin()