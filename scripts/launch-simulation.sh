cd
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch training_coordinator train.launch
