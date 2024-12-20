# quack-norris
## Prerequisites
1. Download and build packages
```
cd ~/code/catkin_ws/user_code/ # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git       # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git   # Clone Apriltag ROS wrapper
git clone git@github.com:maximilianeberlein/quack-norris.git  # Clone this repo
git clone https://github.com/debaumann/quack_norris_utils.git # Clone our utils repos
cd ~/code/catkin_ws        # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y
catkin build apriltag_ros quack-norris quack_norris_utils # Build all relevant packages in the workspace
```

> Note: Installation of rosdep may be necessary. If so, run
```
sudo apt update
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
```

before running the `rosdep install` command.

2. Install pip dependencies
```
pip install shapely pyquaternion
```

## Usage
To launch our main code, including our global and local planners, odometry, apriltag_ros and low-level controllers, run
```
roslaunch quack_norris main_node.lauch
``` 

To test the global planner, run (in separate terminals)
```
rosrun quack_norris map_service_node.py
```

and
```
rosrun quack_norris map_client_node.py
```
