# quack-norris

1. Download and build packages
```
cd ~/code/catkin_ws/src                      # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper
git clone git@github.com:maximilianeberlein/quack-norris.git # Clone this repo
cd ~/code/catkin_ws                          # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y
catkin build apriltag_ros quack-norris   # Build all relevant packages in the workspace
```

> Note: Installation of rosdep may be necessary. If so, run
```
sudo apt update
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
```

before running the `rosdep install` command.

2. Launch AprilTag Detection Node
```
roslaunch quack-norris main_node.launch 
```

