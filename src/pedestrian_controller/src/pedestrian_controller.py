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
    def __init__(self, clientRegistry: ServiceClientRegistry, model_name: str, model_path: str):
        self.service_clients = clientRegistry
        self.model_name = model_name
        self.model_path = model_path

    def spawn(self, initialPose: Pose2D):
        request = SpawnModelRequest(
            yaml_path=self.model_path,
            # only alphanumerics
            name=self.model_name,
            # throws warning if empty, namespaces of all pedestrians have to be different
            ns="pedestrians_" + self.model_name,
            pose=initialPose
        )

        response = self.service_clients.spawn_model(request)
        if not response.success:
            rospy.logerr("Failed to spawn model: %s", response.message)
            exit()

    def move_to_pose(self, pose: Pose2D):
        request = MoveModelRequest(name=self.model_name, pose=pose)

        response = self.service_clients.move_model(request)
        if not response.success:
            rospy.logerr("Failed to move model: %s", response.message)
            exit()

if __name__ == '__main__':
    rospy.init_node('pedestrian_controller')

    # get params
    model_path = rospy.get_param("/pedestrian_controller/pedestrian_model_path")

    service_client_registry = ServiceClientRegistry()

    pedestrian_names = ["pedestrian1", "pedestrian2"]
    pedestrians = [ Pedestrian(service_client_registry, pedestrian_name, model_path) for pedestrian_name in pedestrian_names ]

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
