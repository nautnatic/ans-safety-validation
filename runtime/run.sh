# runs the trained agent

sudo xhost +local:root

export LAUNCH_PACKAGE=arena_bringup
export LAUNCH_FILE=start_arena.launch
export ADDITIONAL_LAUNCH_PARAMS=$(cat <<-END
END
)

docker compose up
