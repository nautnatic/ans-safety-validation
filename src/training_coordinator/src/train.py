#!/usr/bin/env python3

import rospy
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.env_checker import check_env
from flatland.env import FlatlandEnv
from stop_training_callback import StopTrainingCallback


if __name__ == "__main__":
    rospy.init_node('training_coordinator')

    max_steps_per_episode = int(rospy.get_param(
        "/training_coordinator/max_steps_per_episode"))
    behavior_model_output_path: str = rospy.get_param(
        "/training_coordinator/behavior_model_output_path")
    learning_rate = float(rospy.get_param(
        "/training_coordinator/learning_rate"))
    total_timesteps = int(rospy.get_param(
        "/training_coordinator/total_timesteps"))

    env = FlatlandEnv(name="", max_steps_per_episode=max_steps_per_episode)
    check_env(env, warn=True)
    env = DummyVecEnv([lambda: env])
    model = PPO('MlpPolicy', env, verbose=1, learning_rate=learning_rate)
    model.learn(total_timesteps=total_timesteps, callback=StopTrainingCallback(total_timesteps), progress_bar=True)
    model.save(behavior_model_output_path)
    env.close()
