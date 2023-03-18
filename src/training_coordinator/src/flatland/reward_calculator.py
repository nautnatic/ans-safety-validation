class RewardCalculator:
    def __init__(self, env, target_reward=1.0, step_reward=-0.1):
        self.env = env
        self.target_reward = target_reward
        self.step_reward = step_reward

    def __call__(self):
        if self.env.is_target_reached():
            return self.target_reward
        return self.step_reward
