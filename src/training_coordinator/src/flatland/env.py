from gym import Env, spaces
import rospy
import random
import math
import numpy as np
from geometry_msgs.msg import Pose2D, Twist, Vector3
# from flatland_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
# from flatland_msgs.srv import MoveModel, MoveModelRequest, MoveModelResponse
# from flatland_msgs.srv import DeleteModel
from flatland.reward_calculator import RewardCalculator
from flatland.observation_action_spaces import ObservationManager, ActionDecoder, create_action_space
from flatland.model_mediator import FlatlandModelMediator
from flatland.random_pose_generator import RandomBoxPoseGenerator


class FlatlandEnv(Env):
    def __init__(self, name: str = ""):
        super(FlatlandEnv, self).__init__()
        self.name = name

        # create models mediators
        self.robot = FlatlandModelMediator(
            model_name="robot",
            model_path="/home/user/src/training_coordinator/config/environment/models/robot.yaml",
            env=self
        )
        self.target = FlatlandModelMediator(
            model_name="target",
            model_path="/home/user/src/training_coordinator/config/environment/models/target.yaml",
            env=self
        )

        # TODO: cleanup environment if it exists
        # for model in [self.robot, self.target]:
        #     model.delete()

        # setup class methods
        self.observation_manager = ObservationManager(self)
        self.get_observation = self.observation_manager.get_observation
        self.encode_observation = self.observation_manager.encode_observation
        self.decode_action = ActionDecoder()
        self.calculate_reward = RewardCalculator(self)
        self.generate_random_pose = RandomBoxPoseGenerator(
            x_range=[1.0, 30.0], y_range=[1.0, 23.0])

        # create observation and action space
        self.observation_space = self.observation_manager.create_observation_space(
            robot_position_x=[1.0, 30.0],
            robot_position_y=[1.0, 23.0],
            robot_orientation=[-math.pi, math.pi],
            robot_linear_velocity_x=[-20.0, 20.0],
            robot_linear_velocity_y=[-20.0, 20.0],
            robot_angular_velocity=[-20.0, 20.0],
            target_position_x=[1.0, 30.0],
            target_position_y=[1.0, 30.0]
        )
        self.action_space = create_action_space(
            robot_linear_velocity_internal=[-10.0, 10.0],
            robot_angular_velocity_internal=[-10.0, 10.0],
        )

        # spawn models
        self.robot.spawn(initial_pose=self.generate_random_pose())
        self.target.spawn(initial_pose=self.generate_random_pose())

        rospy.sleep(2)

    def close(self):
        rospy.loginfo("Flatland environment closed")
        self.robot.delete()
        self.target.delete()

    def reset(self):
        rospy.loginfo("Reset environment")
        self.robot.move_to_pose(self.generate_random_pose())
        self.target.move_to_pose(self.generate_random_pose())

        rospy.sleep(1)
        observation = self.get_observation()
        encoded_observation = self.encode_observation(observation)

        return encoded_observation

    def step(self, encoded_action):
        # setup for step
        rospy.loginfo("Execute step in environment")
        next_action = self.decode_action(encoded_action)

        # execute action and get observation
        self.robot.set_target_speed(
            linear=next_action["linear"], angular=next_action["angular"])
        rospy.sleep(2)
        observation = self.get_observation()

        # calculate step results
        reward = self.calculate_reward()
        encoded_observation = self.encode_observation(observation)
        done = False
        info = {}

        # reset environment
        if self.is_target_reached():
            self.reset()

        # return step results
        return (
            encoded_observation,
            reward,
            done,
            info
        )

    def is_target_reached(self):
        # TODO
        return False
