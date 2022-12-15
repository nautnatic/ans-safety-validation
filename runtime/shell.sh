docker run -it \
  --mount type="bind",source="${PROJECT_DIR}/catkin_ws",target="/opt/ros/catkin_ws" \
  ros \
  bash