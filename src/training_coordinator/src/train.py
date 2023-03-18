#!/usr/bin/env python3

import rospy
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.env_checker import check_env
from flatland.env import FlatlandEnv


if __name__ == "__main__":
    rospy.init_node('training_coordinator')

    env = FlatlandEnv(name="")
    check_env(env, warn=True)
    env = DummyVecEnv([lambda: env])
    model = PPO('MlpPolicy', env, verbose=1)
    model.learn(total_timesteps=10000)
    model.env.close()
