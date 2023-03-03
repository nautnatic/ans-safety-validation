#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from flatland_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from flatland_msgs.srv import MoveModel, MoveModelRequest, MoveModelResponse
from flatland_msgs.srv import DeleteModel


class ServiceClientRegistry:
    def __init__(self):
        # service clients (first parameter: service, second parameter: service type)
        self.spawn_model = rospy.ServiceProxy('spawn_model', SpawnModel)
        self.move_model = rospy.ServiceProxy('move_model', MoveModel)


class Pedestrian:
    def __init__(self, clientRegistry: ServiceClientRegistry, model_name: str):
        self.service_clients = clientRegistry
        self.model_name = model_name

    def spawn(self, initialPose: Pose2D):
        request = SpawnModelRequest(
            yaml_path="/home/user/src/flatland_agent_spawner/config/turtlebot_model.yaml",
            # only alphanumerics
            name=self.model_name,
            # throws warning if empty, namespaces of all pedestrians have to be different
            ns="pedestrians_" + self.model_name,
            pose=initialPose
        )

        response = self.service_clients.spawn_model(request)
        if not response.success:
            rospy.logerr("Failed to spawn model: %s", response.error_message)

    def move_to_pose(self, pose: Pose2D):
        request = MoveModelRequest(name=self.model_name, pose=pose)

        response = self.service_clients.move_model(request)
        if not response.success:
            rospy.logerr("Failed to move model: %s", response.error_message)


if __name__ == '__main__':
    rospy.init_node('flatland_agent_spawner')
    service_client_registry = ServiceClientRegistry()

    pedestrian_names = ["pedestrian1", "pedestrian2"]
    pedestrians = [ Pedestrian(service_client_registry, ped_name) for ped_name in pedestrian_names ]

    initial_pose = Pose2D(x=0.0, y=0.0, theta=0.0)
    for pedestrian in pedestrians:
        pedestrian.spawn(initial_pose)

	# rate in Hertz
    rate = rospy.Rate(2)
    toggle = False

    while not rospy.is_shutdown():
        if toggle:
            new_pose = Pose2D(x=10.0, y=0.0, theta=0.0)
        else:
            new_pose = Pose2D(x=-10.0, y=0.0, theta=0.0)

        pedestrians[0].move_to_pose(new_pose)
        # rospy.loginfo("moved to new pos")

        toggle = not toggle
        rate.sleep()
