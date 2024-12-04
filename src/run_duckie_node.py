#!/usr/bin/env python3

import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from duckietown_msgs.msg import WheelsCmdStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse

from typing import Any

# TODO: All to edit with actual commands and "Move" commands, see old commented file below
class RunDuckieRviz:
    def __init__(self):
        """
        Initialize the RunDuckieRviz class, set up ROS node, parameters, publishers, and subscribers.
        """
        # Initialize the ROS node
        rospy.init_node("rviz_node", anonymous=True)

        # init parameters
        self.duckiebot_name = "sim_duckiebot"

        self.setup_params()
        self.create_publishers()
        self.create_subscribers()

        # define current pose
        self.cur_pose = None

        # initialize observed path msg
        # self.init_obs_path()
        # initialize list for observed curves

        # set update rate
        self.update_rate = 60
        self.rate = rospy.Rate(self.update_rate)

    def setup_params(self) -> None:
        """
        Set up the parameters for the ROS topics.

        Returns:
            None
        """

        self.topic_prefix = "/" + self.duckiebot_name

        def get_rosparam(name: str) -> Any:
            """
            Helper function to retrieve a ROS parameter.

            Args:
                name (str): The name of the ROS parameter.

            Returns:
                Any: The value of the ROS parameter.

            Raises:
                KeyError: If the ROS parameter is not set.
            """
            if rospy.has_param(name):
                param = rospy.get_param(name)
            else:
                txt_error = f"Parameter '{name}' is not set."
                rospy.logerr(txt_error)
                raise KeyError(txt_error)
            return param

        # topics params
        self.name_sub_sim_pose_topic = get_rosparam("~topics/sub/sim_pose/name")
        self.name_wheel_cmd_pub_topic = get_rosparam("~topics/pub/wheels_cmd/name")
        # self.name_gt_path_pub_topic = get_rosparam("~topics/pub/gt_path/name")
        # self.name_observed_path_pub_topic = get_rosparam("~topics/pub/obs_path/name")

    def create_publishers(self) -> None:
        """
        Create the ROS publishers.
        """
        self.wheel_cmd_pub = rospy.Publisher(self.name_wheel_cmd_pub_topic, WheelsCmdStamped, queue_size=10)
        # self.gt_path_pub = rospy.Publisher(self.name_gt_path_pub_topic, Path, queue_size=10)
        # self.observed_path_pub = rospy.Publisher(self.name_observed_path_pub_topic, Path, queue_size=10)

    def create_subscribers(self) -> None:
        """
        Create the ROS subscribers.
        """
        self.sim_pose_sub = rospy.Subscriber(self.name_sub_sim_pose_topic, PoseStamped, self.sim_pose_callback)

    # def init_obs_path(self) -> None:
    #     """
    #     Initialize the observed path message.
    #     """
    #     self.observed_path = Path()
    #     self.observed_path.header = Header()
    #     self.observed_path.header.frame_id = "world"

    def sim_pose_callback(self, msg: PoseStamped) -> None:
        """
        Callback function for the simulated pose subscriber.

        Args:
            msg (PoseStamped): The message containing the pose data.
        """
        self.cur_pose = msg.pose
        # Since the robot should remain still, we do not process the pose further.

    def run(self) -> None:
        """
        Run the node to execute the commands and control the robot.
        """
        while not rospy.is_shutdown():
            # Calculate the wheel commands
            wheel_cmd = WheelsCmdStamped()
            wheel_cmd.header.stamp = rospy.Time.now()
            wheel_cmd.vel_left = 0.0
            wheel_cmd.vel_right = 0.0
            self.wheel_cmd_pub.publish(wheel_cmd)
            self.rate.sleep()




















# #!/usr/bin/env python3

# import rospy

# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
# from duckietown_msgs.msg import WheelsCmdStamped
# from nav_msgs.msg import Path
# from tf.transformations import euler_from_quaternion
# import numpy as np
# from std_msgs.msg import Header
# from std_srvs.srv import Trigger, TriggerResponse

# # from src.move import Move
# # from include.visualization import GtVis
# from typing import Any


# class RunDuckieRviz:
#     def __init__(self, chosen_shape: str, commands: list):
#         """
#         Initialize the Exercise1Solution class, set up ROS node, parameters, publishers, and subscribers.

#         Args:
#             chosen_shape (str): The shape to be executed.
#             commands (list): A list of commands to be executed by the robot.
#         """
#         # Initialize the ROS node
#         rospy.init_node("rviz_node", anonymous=True)

#         self.shape = chosen_shape
#         self.commands = commands

#         # init parameters
#         self.duckiebot_name = "sim_duckiebot"

#         self.setup_params()
#         self.create_publishers()
#         self.create_subscribers()

#         # define current pose
#         self.cur_pose = None

#         # initialize observed path msg
#         self.init_obs_path()
#         # initialize list for observed curves
#         self.observed_curve = []

#         # set update rate
#         self.update_rate = 60
#         self.rate = rospy.Rate(self.update_rate)

#     def setup_params(self) -> None:
#         """
#         Set up the parameters for the ROS topics.

#         Returns:
#             None
#         """

#         self.topic_prefix = "/" + self.duckiebot_name

#         def get_rosparam(name: str) -> Any:
#             """
#             Helper function to retrieve a ROS parameter.

#             Args:
#                 name (str): The name of the ROS parameter.

#             Returns:
#                 Any: The value of the ROS parameter.

#             Raises:
#                 KeyError: If the ROS parameter is not set.
#             """
#             if rospy.has_param(name):
#                 param = rospy.get_param(name)
#             else:
#                 txt_error = f"Parameter '{name}' is not set."
#                 rospy.logerr(txt_error)
#                 raise KeyError(txt_error)
#             return param

#         # topics params
#         self.name_sub_sim_pose_topic = get_rosparam("~topics/sub/sim_pose/name")
#         self.name_wheel_cmd_pub_topic = get_rosparam("~topics/pub/wheels_cmd/name")
#         self.name_gt_path_pub_topic = get_rosparam("~topics/pub/gt_path/name")
#         self.name_observed_path_pub_topic = get_rosparam("~topics/pub/obs_path/name")



#     def create_publishers(self) -> None:
#         """
#         Create the ROS publishers.

#         Returns:
#             None
#         """
#         # publish wheel command
#         self.wheel_cmd_pub = rospy.Publisher(
#             self.topic_prefix + self.name_wheel_cmd_pub_topic,
#             WheelsCmdStamped,
#             queue_size=10,
#         )
#         # publish ground truth path
#         self.gt_path_pub = rospy.Publisher(
#             self.topic_prefix + self.name_gt_path_pub_topic, Marker, queue_size=10
#         )
#         # publish observed Path
#         self.observed_path_pub = rospy.Publisher(
#             self.topic_prefix + self.name_observed_path_pub_topic, Path, queue_size=10
#         )


#     def create_subscribers(self) -> None:
#         """
#         Create the ROS subscribers.

#         Returns:
#             None
#         """
#         # Subscribe to the ground truth Pose topic
#         self.pose_sub = rospy.Subscriber(
#             self.topic_prefix + self.name_sub_sim_pose_topic,
#             PoseStamped,
#             self.pose_callback,
#         )


#     def init_obs_path(self) -> None:
#         """
#         Initialize the message for the observed path.

#         Returns:
#             None
#         """
#         self.observed_path_msg = Path()
#         self.observed_path_msg.header = Header()
#         self.observed_path_msg.header.frame_id = "world"


#     def pose_callback(self, msg: PoseStamped) -> None:
#         """
#         Callback function to update the current pose and publish the observed path.

#         Args:
#             msg (PoseStamped): The current pose message.

#         Returns:
#             None
#         """
#         # update current pose
#         self.cur_pose = msg.pose
#         # publish observed path
#         self.observed_path_msg.poses.append(msg)
#         self.observed_path_pub.publish(self.observed_path_msg)
#         # add pose to curve
#         self.observed_curve.append(Point(msg.pose.position.x, msg.pose.position.y, 0.0))

#     def wait_for_starting_pose(self) -> None:
#         """
#         Wait for the current pose to be available before starting the execution of commands.

#         Returns:
#             None
#         """
#         if self.cur_pose is None:
#             rospy.loginfo(
#                 f"Waiting to receive from the simulation the topic {self.topic_prefix + self.name_sub_sim_pose_topic}"
#             )

#         while self.cur_pose is None:
#             rospy.sleep(1)

#         rospy.loginfo(
#             f"Received topic {self.topic_prefix + self.name_sub_sim_pose_topic}! Starting to execute commands."
#         )


#     def run(self) -> None:
#         """
#         Run the node to execute the commands and control the robot.

#         Returns:
#             None
#         """
#         # wait until starting pose is available
#         self.wait_for_starting_pose()
#         # get ground truth path
#         self.gt_vis = GtVis(self.shape, self.cur_pose, self.commands)
#         gt_path = self.gt_vis.get_gt_path()
#         # init observed path
#         self.init_obs_path()
#         # init move class
#         self.move = Move(self.cur_pose, self.commands)
#         # run at rate
#         while not rospy.is_shutdown() and not self.move.all_commands_excecuted:
#             # publish gt path
#             self.gt_path_pub.publish(gt_path)
#             # calculate wheel cmd
#             wheel_cmd = self.move.get_wheel_cmd(self.cur_pose)
#             self.wheel_cmd_pub.publish(wheel_cmd)
#             self.rate.sleep()
