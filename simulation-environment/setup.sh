cd $ARENA_ROSNAV_REPO
rosws update
source $HOME/.zshrc
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.zsh