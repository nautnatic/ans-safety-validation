LAUNCH_FILE=start_arena_flatland.launch

roslaunch arena_bringup $LAUNCH_FILE \
	# true/false
	train_mode:=false
	# true/false; default=true
	use_viz:=false
	# teb/dwa/mpc/cadrl/arena2d; default=dwa
	local_planner:=false
	# maximum velocity of dynamic obstacles in [m/s]; recommended range: [0.1,0.7]; default=0.3
	obs_vel:=false
	# default: map_empty
	map_file:=false
