docker run -it \
  --mount type="bind",source="${PROJECT_DIR}/catkin_ws",target="/opt/ros/catkin_ws" \
  ros \
  ros2 launch ${LAUNCH_PACKAGE} ${LAUNCH_FILE} ${ADDITIONAL_LAUNCH_PARAMS}