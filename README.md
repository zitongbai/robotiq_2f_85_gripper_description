# robotiq_2f_85_gripper_description
ROS2 package for Robotiq 85mm 2-Finger-Adaptive-Gripper

Some resources like urdf, meshes, etc. are from [ros-industrial/robotiq](https://github.com/ros-industrial/robotiq). 

# Environment

* Ubuntu 22.04
* ROS2 humble

# Usage

Download and build: 
```shell
mkdir -r ros2_ws/src
cd ros2_ws/src
git clone git@github.com:zitongbai/robotiq_2f_85_gripper_description.git
cd ..
colcon build
```
Visualize in rviz:
```shell
ros2 launch robotiq_2f_85_gripper_description view_gripper.launch.py
```

remember to `source install/setup.bash`

