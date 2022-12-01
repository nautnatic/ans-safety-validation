# Setup of Arena Rosnav simulation environment inside a Container
## Preconditions
1. Operating system = Linux
2. Docker runtime setup
3. docker-compose and docker CLI installed on the development machine

## Clone arena-rosnav Git repository
The following [repository](https://github.com/ignc-research/arena-rosnav) should be cloned to your machine. Https should be the preferred way to clone, because then the pull works from inside the container as well without setting up ssh.

	git clone https://github.com/ignc-research/arena-rosnav.git

## Start & stop of the containers
The environment consists of 2 containers: The *ros* container contains the actual runtime environment. The *novnc* container provides a web GUI on the following [link](http://localhost:8080/vnc.html). 

The containers can be started by executing the script *setup-environment.sh*, which starts the containers as defined in the *docker-compose.yaml*
To teardown the containers, execute *teardown-environment.sh*

However, before running the command you need to export the environment variable PEPO_PATH either manually or automatically with *direnv*. REPO_PATH should be set to the path of the arena-rosnav repository.

Additionally, please make sure that the forwarded ports and volume mount paths are not blocked on your host machine.

# Interacting with the container
## Enter the shell directly
The following command executes the shell in the ros container, where you can execute the *roslaunch* command to manage the simulation.

	docker exec -it -w $ARENA_ROSNAV_MOUNTED ros /bin/zsh

Please note that you have to define the environment variable ARENA_ROSNAV_MOUNTED before execution.

You can exit the container shell with the following command:

	exit

## Remote development in container with VS Code
Install the *Remote Development* extension of Microsoft in VS Code.

Afterwards the following action can be executed in VS Code: 
	
	Remote Development: Attach to running container /ros


# Start simulation
In the docker-compose the arena-rosnav repository is mounted as a volume into the ros container at the path defined in the environment variable ARENA_ROSNAV_MOUNTED

	/root/arena_ws/src/arena-rosnav
	
To run the simulation the following commands have to be executed:

	# activate virtual environment
	workon rosnav

	# update git repo
	cd /root/arena_ws/src/arena-rosnav
	git pull

	# update ROS workspace
	rosws update

	# build your workspace
	cd /root/arena_ws
	catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3

	# launch the simulation
	roslaunch arena_bringup <launchFileName> <launchOptions>
	
A detailed description of all relevant launch parameters can be found in *launch-simulation.sh*.
A simple example of launch command, which uses only the default launch options would be:

	roslaunch arena_bringup start_arena_flatland.launch

A more advanced example of the launch command would be:

	roslaunch arena_bringup start_arena_flatland.launch local_planner:="rlca" map_file:="map1"  disable_scenario:="false" scenario_file:="eval/obstacle_map1_obs20.json"

# Setup of environment on Ubuntu 20+

	# install poetry
	curl -sSL https://install.python-poetry.org | python3 -

	# install nlopt
	git clone https://github.com/stevengj/nlopt
	cd nlopt
	mkdir build
	cd build
	cmake ..
	make
	sudo make install





# Error handling
Common errors and their solution are described in the repository at the following [link](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/guide.md#common-error-handling).