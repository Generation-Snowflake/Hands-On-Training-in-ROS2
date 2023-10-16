# Hands-On-Training-in-ROS2

## Installation
#### ROS2 PKGS
- sudo apt-get install ros-foxy-joint-state-publisher
- sudo apt-get install ros-foxy-joint-state-controller
- sudo apt-get install ros-foxy-robot-state-publisher
- sudo apt-get install ros-foxy-gazebo-ros2-control
- sudo apt install ros-foxy-ros-core ros-foxy-geometry2
###### galactic
- sudo apt-get install ros-galactic-gazebo-ros
- sudo apt-get install ros-galactic-gazebo-ros2-control
- sudo apt-get install ros-galactic-joint-state-publisher
- sudo apt-get install ros-galactic-xacro
- sudo apt-get install ros-galactic-nav2-bringup
- sudo apt-get install ros-galactic-navigation2 
- sudo apt-get install ros-galactic-gazebo-plugins
- sudo apt-get install ros-galactic-nav2-simple-commander
- sudo apt-get install ros-galactic-tf-transformations 
- sudo pip3 install transforms3d
- sudo apt-get install ros-galactic-cartographer


#### Create pkg and build
- Create YOUR_WS by ``mkdir <YOUR_WS>/src``
- Get into YOUR_WS/src
- Download pkgs Hands-On-Training-in-ROS2 by ``git clone https://github.com/Generation-Snowflake/Hands-On-Training-in-ROS2.git``
- Back to YOUR_WS directory ``cd ..``
- Build pkgs by ``colcon build --symlink-install``
## Getting Started
#### Run Mecanum Robot Description with Rviz
- ``ros2 launch mecanum_description mecanum_description.launch.py rviz:=true``
#### Run Mecanum Robot in Gazebo with joy
- Laucnh robot description and gazebo empty world ``ros2 launch mecanum_gazebo mecanum_empty_world.launch.py``
- Run teleop twist keyboard to control Mecanum robot ``ros2 run teleop_twist_keyboard teleop_twist_keyboard``


## Ref
- https://github.com/linorobot/linorobot2
- Mecanum formula -  https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
#### Encoder and SpeedControl
- https://www.pjrc.com/teensy/td_libs_Encoder.html
- https://github.com/phuwanat-vg/ros2_tutorial/blob/master/robot_firmware