# Setup of Arena Rosnav simulation environment inside a Container
## Preconditions
1. Docker runtime setup
2. docker-compose and docker CLI installed on the development machine

## Get arena-rosnav Git repository
The following [repository](https://github.com/ignc-research/arena-rosnav) should be cloned to your machine, e.g. through ssh:

	git clone git@github.com:ignc-research/arena-rosnav.git

## Start & stop of the containers
The environment consists of 2 containers: The *ros* container contains the actual runtime environment. The *novnc* container provides a web GUI on the following [link](http://localhost:8080/vnc.html).

They containers can be started through the attached docker-compose by executing the following command in the directory with the docker-compose:
	
	docker-compose up -d

To stop the containers execute the following command:

	docker-compose down -d

However, before running the command you need to export the environment variable PEPO_PATH either manually or automatically with *direnv*. REPO_PATH should be set to the path of the arena-rosnav repository.


# Interacting with the container
## Enter the shell directly
The following command executes the shell in the ros container, where you can execute the *roslaunch* command to manage the simulation.

	docker exec -it -w $ARENA_ROSNAV_MOUNTED ros /bin/zsh

You can exit the container shell with the following command:

	exit

## Remote development in container with VS Code
Install the *Remote Development* extension of Microsoft in VS Code.

Afterwards the following action can be executed in VS Code: 
	
	Remote Development: Attach to running container /ros


# Start simulation
In the docker-compose the arena-rosnav repository is mounted as a volume into the ros container at the following path:

	/root/arena_ws/src/arena-rosnav
	
To run the simulation the following commands have to be executed:

	# activate virtual environment
	workon rosnav

	# update git repo
	cd /root/arena_ws/src/arena-rosnav
	git pull

	# update ROS workspace
	rosws update

	# Build your workspace
	cd /root/arena_ws
	catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3

	roslaunch arena_bringup start_arena_flatland.launch  train_mode:=false

	roslaunch arena_bringup <launchFileName>
	roslaunch arena_bringup start_arena_flatland.launch

## Roslaunch Parameters
	train_mode:=<true, false>
	use_viz:=<true, false> (default true)
	local_planner:=<teb,dwa,mpc,cadrl,arena2d> (default dwa)
	task_mode:=<random, manual, scenario> (default random) (redundant and does not need to be specified anymore)
	obs_vel:= # maximum velocity of dynamic obstacles [m/s]. It is recommended to set a max velocity within [0.1,0.7] (default 0.3)
	map_file:= # e.g. map1 (default map_empty)

## Example for roslaunch command
	roslaunch arena_bringup start_arena_flatland.launch local_planner:="rlca" map_file:="map1"  disable_scenario:="false" scenario_file:="eval/obstacle_map1_obs20.json"

# Error handling
Common errors and their solution are described in the repository at the following [link](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/guide.md#common-error-handling).