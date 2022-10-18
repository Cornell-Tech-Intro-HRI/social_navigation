cd ~/<ros_workspace>/
source /opt/ros/galactic/setup.bash 
colcon build 
source install/local_setup.bash
ros2 run social_navigation_pkg social_navigation_node