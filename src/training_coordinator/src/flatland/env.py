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
from flatland.observation_action_spaces import (
    ObservationManager,
    ActionDecoder,
    create_action_space,
)
from flatland.model_mediator import FlatlandModelMediator
from flatland.random_pose_generator import RandomBoxPoseGenerator


class FlatlandEnv(Env):
    def __init__(self, name: str = ""):
        super(FlatlandEnv, self).__init__()
        self.name = name
        self.current_episode = 0
        # steps only counted within one episode
        self.current_step = 0
        self.max_steps_per_episode = 20

        # create models mediators
        self.robot = FlatlandModelMediator(
            model_name="robot",
            model_path="/home/user/src/training_coordinator/config/environment/models/robot.yaml",
            env=self,
        )
        self.target = FlatlandModelMediator(
            model_name="target",
            model_path="/home/user/src/training_coordinator/config/environment/models/target.yaml",
            env=self,
        )

        # setup class methods
        self.observation_manager = ObservationManager(self)
        self.get_observation = self.observation_manager.get_observation
        self.encode_observation = self.observation_manager.encode_observation
        self.decode_action = ActionDecoder()
        self.calculate_reward = RewardCalculator(
            env=self,
            target_reward=1.0,
            step_base_reward=-1.0,
            distance_factor=-0.5,
            distance_threshold=1.0,
        )
        self.spawn_strategy = RandomBoxPoseGenerator(
            x_range=[1.0, 30.0], y_range=[1.0, 23.0]
        )

        # create observation and action space
        self.observation_space = self.observation_manager.create_observation_space(
            robot_position_x=[1.0, 30.0],
            robot_position_y=[1.0, 23.0],
            robot_orientation=[-math.pi, math.pi],
            robot_linear_velocity_x=[-20.0, 20.0],
            robot_linear_velocity_y=[-20.0, 20.0],
            robot_angular_velocity=[-20.0, 20.0],
            target_position_x=[1.0, 30.0],
            target_position_y=[1.0, 30.0],
        )
        self.action_space = create_action_space(
            robot_linear_velocity_internal=[-5.0, 5.0],
            robot_angular_velocity_internal=[-5.0, 5.0],
        )

        # spawn models
        self.robot.spawn(initial_pose=self.spawn_strategy.get_pose())
        self.target.spawn(initial_pose=self.spawn_strategy.get_pose())

        rospy.sleep(2)

    def close(self):
        rospy.loginfo("Flatland environment closed")
        self.robot.delete()
        self.target.delete()

    def reset(self):
        rospy.loginfo("Reset environment")
        self.current_step = 0
        self.current_episode = self.current_episode + 1

        self.robot.move_to_pose(self.spawn_strategy.get_pose())
        self.target.move_to_pose(self.spawn_strategy.get_pose())

        rospy.sleep(0.5)
        observation = self.get_observation()
        encoded_observation = self.encode_observation(observation)

        return encoded_observation

    def step(self, encoded_action):
        # setup for step
        next_action = self.decode_action(encoded_action)

        # execute action and get observation
        self.robot.set_target_speed(
            linear=next_action["linear"], angular=next_action["angular"]
        )
        rospy.sleep(0.5)
        observation = self.get_observation()

        # calculate step results
        reward = self.calculate_reward(observation)
        encoded_observation = self.encode_observation(observation)
        info = {}
        
        # check if episode is done
        if reward == 1.0 or self.current_step == self.max_steps_per_episode:
            done = True
        else:
            done = False

        self.current_step = self.current_step + 1

        rospy.loginfo(f"| Episode {self.current_episode:3d} | Step {self.current_step:3d} | -> Reward: {reward:6.2f}")

        # return step results
        return (encoded_observation, reward, done, info)
