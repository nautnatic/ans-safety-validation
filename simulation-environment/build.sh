workon rosnav
cd $ARENA_ROSNAV_REPO
rosws update
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.zsh