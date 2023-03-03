cd
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch pedestrian_controller pedestrian_controller.launch
