import math


class RewardCalculator:
    def __init__(self, env, target_reward, step_base_reward, distance_factor, distance_threshold):
        self.env = env
        self.target_reward = target_reward
        self.step_base_reward = step_base_reward
        self.distance_factor = distance_factor
        self.distance_threshold = distance_threshold

    def __call__(self, observation):
        distance = self._get_euclidean_distance()
        if distance < self.distance_threshold:
            return self.target_reward
        return self.step_base_reward + self.distance_factor * distance

    def _get_euclidean_distance(self):
        obs = self.env.get_observation()
        return math.sqrt(
            (obs["robot"]["position"]["x"] - obs["target"]["position"]["x"]) ** 2 +
            (obs["robot"]["position"]["y"] - obs["target"]["position"]["y"]) ** 2
        )
