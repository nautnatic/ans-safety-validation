#!/usr/bin/env python3

import random
import rospy
from geometry_msgs.msg import Pose2D, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from flatland_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from flatland_msgs.srv import MoveModel, MoveModelRequest, MoveModelResponse
from flatland_msgs.srv import DeleteModel


class ServiceClientRegistry:
    def __init__(self):
        # service clients (first parameter: service, second parameter: service type)
        self.spawn_model = rospy.ServiceProxy('spawn_model', SpawnModel)
        self.move_model = rospy.ServiceProxy('move_model', MoveModel)


PEDESTRIAN_NAMESPACE_PREFIX = "ped_"


class OdometrySubscriptionHandler:
    def __init__(self):
        self.pos_x_before = 0.0

    def handle_odometry_subscription_message(self, odometry: Odometry):
        pose = odometry.pose.pose
        if pose.position.x != self.pos_x_before:
            self.pos_x_before = pose.position.x
            rospy.loginfo(rospy.get_caller_id() + 'I heard %s', odometry)


if __name__ == '__main__':
    service_clients = ServiceClientRegistry()

    model_path = "$(find training_coordinator)/config/environment/models/pedestrian.yaml"
    name = "myname3"
    topic_root = PEDESTRIAN_NAMESPACE_PREFIX + name
    initial_pose = Pose2D(x=10.0, y=10.0, theta=0.0)

    # spawn model
    request = SpawnModelRequest(
        yaml_path=model_path,
        name=name,
        ns=topic_root,
        pose=initial_pose
    )
    response = service_clients.spawn_model(request)
    if not response.success:
        rospy.logerr("Failed to spawn model: %s", response.message)
        exit()