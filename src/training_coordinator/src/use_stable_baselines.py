#!/usr/bin/env python3

from stable_baselines3 import PPO
from stable_baselines3.common.on_policy_algorithm import OnPolicyAlgorithm
from stable_baselines3.common.env_util import make_vec_env


def setup_environments():
    # environment_parameters = {"param1": 1}
    # return make_vec_env('CartPole-v1', n_envs=4, env_kwargs=environment_parameters)
    return make_vec_env('CartPole-v1', n_envs=4)


def build_new_model(environments) -> OnPolicyAlgorithm:
    return PPO('MlpPolicy', environments, verbose=1)


def load_pretrained_model():
    raise NotImplementedError()


if __name__ == "__main__":
    use_pretrained_model = False

    environments = setup_environments()

    if use_pretrained_model:
        model = load_pretrained_model()
    else:
        model = build_new_model(environments)

    # trains the model using all the previously defined environments
    model.learn(total_timesteps=100000)
