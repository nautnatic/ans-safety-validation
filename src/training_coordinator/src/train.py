#!/usr/bin/env python3

import rospy
import os
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import CheckpointCallback
from flatland.env import FlatlandEnv
from utils.stop_training_callback import StopTrainingCallback
from utils.LogManager import LogManager


if __name__ == "__main__":
    rospy.init_node('training_coordinator')

    max_steps_per_episode = int(rospy.get_param(
        "/training_coordinator/max_steps_per_episode"))
    log_container_path: str = rospy.get_param(
        "/training_coordinator/log_container_path")
    learning_rate = float(rospy.get_param(
        "/training_coordinator/learning_rate"))
    total_timesteps = int(rospy.get_param(
        "/training_coordinator/total_timesteps"))
    checkpoint_frequency_in_steps = int(rospy.get_param(
        "/training_coordinator/checkpoint_frequency_in_steps"))
    number_logs_to_keep = int(rospy.get_param(
        "/training_coordinator/number_logs_to_keep"))

    log_manager = LogManager(log_container_path)
    log_manager.keep_last_x_logs(number_logs_to_keep - 1)
    log_path = log_manager.create_next_log_path()

    training_callbacks = [
        StopTrainingCallback(total_timesteps),
        CheckpointCallback(save_freq=checkpoint_frequency_in_steps,
                           save_path=os.path.join(log_path, "checkpoints"),
                           name_prefix="model")
    ]

    env = FlatlandEnv(name="", max_steps_per_episode=max_steps_per_episode)
    check_env(env, warn=True)
    env = DummyVecEnv([lambda: env])
    model = PPO('MlpPolicy', env, verbose=1, learning_rate=learning_rate)
    model.learn(total_timesteps=total_timesteps, callback=training_callbacks, progress_bar=True)
    model.save(os.path.join(log_path, "model_finished"))
    env.close()
