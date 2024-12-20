# quack-norris
#prerequisites
- ros image pipeline
- shapely
- quack_norris_utils
1. Download and build packages
```
cd ~/code/catkin_ws/src                      # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper
git clone git@github.com:maximilianeberlein/quack-norris.git # Clone this repo
cd ~/code/catkin_ws        # Navigate to the workspace
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
3. Launch dubins solver and controller

   this relies on the quack_norris_utils get it here and catkin build it
  ```
   cd catkin_ws/src/user_code/
   git clone https://github.com/debaumann/quack_norris_utils.git
   catkin build quack_norris_utils 
   
  ```
