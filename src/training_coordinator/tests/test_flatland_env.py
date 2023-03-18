#!/usr/bin/env python3

import rospy
from flatland.env import FlatlandEnv
from stable_baselines3.common.env_checker import check_env
import numpy as np


if __name__ == '__main__':
    rospy.init_node('training_coordinator')
    env = FlatlandEnv()
    check_env(env, warn=True)

    env.reset()
    rospy.sleep(2)
    env.reset()
    rospy.sleep(2)

    obs = env.step(np.array([5.0, 5.0]))
    rospy.loginfo(obs)

    rospy.sleep(5)
    env.close()
