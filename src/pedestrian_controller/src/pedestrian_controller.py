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


class Pedestrian:
    def __init__(self, service_client_registry: ServiceClientRegistry, name: str, model_path: str, ground_truth_handler):
        self.service_clients = service_client_registry
        self.name = name
        self.model_path = model_path

        self.topic_root = PEDESTRIAN_NAMESPACE_PREFIX + self.name

        # subscribers
        ground_truth_subscriber = rospy.Subscriber(
            f'/{self.topic_root}/odometry/ground_truth', Odometry, ground_truth_handler)

        # publishers
        self.speed_publisher = rospy.Publisher(
            f'/{self.topic_root}/cmd_vel', Twist, queue_size=10)

    def spawn(self, initialPose: Pose2D):
        request = SpawnModelRequest(
            yaml_path=self.model_path,
            # only alphanumerics
            name=self.name,
            # throws warning if empty, namespaces of all pedestrians have to be different
            ns=self.topic_root,
            pose=initialPose
        )

        response = self.service_clients.spawn_model(request)
        if not response.success:
            rospy.logerr("Failed to spawn model: %s", response.message)
            exit()

    def move_to_pose(self, pose: Pose2D):
        request = MoveModelRequest(name=self.name, pose=pose)

        response = self.service_clients.move_model(request)
        if not response.success:
            rospy.logerr("Failed to move model: %s", response.message)
            exit()

    # def delete_model(self):
    #     # TODO
    #     ...

    def set_speed(self, twist: Twist):
        self.speed_publisher.publish(twist)


class OdometrySubscriptionHandler:
    def __init__(self):
        self.pos_x_before = 0.0

    def handle_odometry_subscription_message(self, odometry: Odometry):
        pose = odometry.pose.pose
        if pose.position.x != self.pos_x_before:
            self.pos_x_before = pose.position.x
            rospy.loginfo(rospy.get_caller_id() + 'I heard %s', odometry)


def get_random_twist():
    def rand():
        return random.uniform(0,10)

    return Twist(
        linear=Vector3(x=rand(), y=rand(), z=rand()),
        angular=Vector3(x=rand(), y=rand(), z=rand())
    )


if __name__ == '__main__':
    rospy.init_node('pedestrian_controller')

    # get params
    model_path: str = rospy.get_param(
        "/pedestrian_controller/pedestrian_model_path")
    pedestrian_count: int = rospy.get_param(
        "/pedestrian_controller/pedestrian_count")

    # setup
    service_client_registry = ServiceClientRegistry()
    odometryHandler = OdometrySubscriptionHandler()
    pedestrians = [
        Pedestrian(
            service_client_registry=service_client_registry,
            name=f"pedestrian{pedestrian_number}",
            model_path=model_path,
            ground_truth_handler=odometryHandler.handle_odometry_subscription_message)
        for pedestrian_number in range(pedestrian_count)
    ]

    # spawn pedestrians
    initial_pose = Pose2D(x=0.0, y=0.0, theta=0.0)
    for pedestrian in pedestrians:
        pedestrian.spawn(initial_pose)

    # rate in Hertz
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        for pedestrian in pedestrians:
            twist = get_random_twist()
            rospy.loginfo(f"Set speed of {pedestrian.name} to {twist}")
            pedestrian.set_speed(twist)
        rate.sleep()

    rospy.spin()


# Topics
# /pedestrians_pedestrian0/cmd_vel
# /pedestrians_pedestrian0/odom -> pose (position & orientation) and velocity (linear and angular)
# /pedestrians_pedestrian0/odometry/ground_truth -> most accurate odometry measurement
# /pedestrians_pedestrian0/scan
# /pedestrians_pedestrian0/scan_3d
# /pedestrians_pedestrian0/twist -> orientation
