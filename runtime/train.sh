# start the environment in training mode
sudo xhost +local:root

export LAUNCH_PACKAGE=arena_bringup
export LAUNCH_FILE=start_training.launch
export ADDITIONAL_LAUNCH_PARAMS=$(cat <<-END
train_mode:=true
END
)

docker compose up
