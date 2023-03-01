cd
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch flatland_agent_spawner launch_and_spawn.launch
