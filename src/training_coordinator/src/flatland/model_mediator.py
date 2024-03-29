import rospy
from geometry_msgs.msg import Pose2D, Twist, Vector3
from nav_msgs.msg import Odometry
from flatland_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from flatland_msgs.srv import MoveModel, MoveModelRequest, MoveModelResponse
from flatland_msgs.srv import DeleteModel, DeleteModelRequest


class FlatlandModelMediator:
    """
    Contains methods to publish to topics or call services of this Flatland environment
    """

    def __init__(self, model_name, model_path, env):
        self.model_name = model_name
        self.env_name = env.name
        self.model_path = model_path
        self._last_odometry = None

        # services
        self._spawn_model_service = rospy.ServiceProxy(
            'spawn_model', SpawnModel)
        self._delete_model_service = rospy.ServiceProxy(
            'delete_model', DeleteModel)
        self._move_model_service = rospy.ServiceProxy(
            'move_model', MoveModel)

        # publishers
        self._set_model_speed_publisher = rospy.Publisher(
            f'{model_name}/cmd_vel', Twist, queue_size=10)

        # subscribers
        def handle_odom_subscription(odom: Odometry):
            self._last_odometry = odom
        self._ground_truth_subscriber = rospy.Subscriber(
            f'{model_name}/odometry/ground_truth', Odometry, handle_odom_subscription)

    def spawn(self, initial_pose: Pose2D):
        request = SpawnModelRequest(
            yaml_path=self.model_path,
            # only alphanumerics
            name=self.model_name,
            # throws warning if empty, namespaces of all pedestrians have to be different
            ns=self.model_name,
            pose=initial_pose
        )
        response = self._spawn_model_service(request)
        if not response.success:
            rospy.logerr("Failed to spawn model: %s", response.message)
            exit()

    def move_to_pose(self, target_pose: Pose2D):
        request = MoveModelRequest(
            # only alphanumerics
            name=self.model_name,
            # throws warning if empty, namespaces of all pedestrians have to be different
            pose=target_pose
        )
        response = self._move_model_service(request)
        if not response.success:
            rospy.logerr("Failed to move model: %s", response.message)
            exit()

    def delete(self):
        response = self._delete_model_service(self.model_name)
        if not response.success:
            rospy.logerr("Failed to delete model: %s", response.message)
            exit()

    def set_target_speed(self, linear: float, angular: float):
        twist = Twist(
            linear=Vector3(x=linear, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=angular)
        )
        self._set_model_speed_publisher.publish(twist)

    def get_odometry(self):
        while self._last_odometry is None:
            rospy.sleep(0.2)
        return self._last_odometry
