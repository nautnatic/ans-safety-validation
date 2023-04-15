# set velocity
rostopic pub /robot/cmd_vel geometry_msgs/Twist "{linear: {x: 5.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 5.0} }" -1

# call step_world
rosservice call /step_world