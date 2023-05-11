from stable_baselines3.common.callbacks import BaseCallback


class StopTrainingCallback(BaseCallback):
    """
    Callback to stop training after a certain number of timesteps.
    """
    def __init__(self, total_timesteps):
        super(StopTrainingCallback, self).__init__()
        self.total_timesteps = total_timesteps

    def _on_training_start(self) -> None:
        """
        This method is called before the first rollout starts.
        """
        self.num_timesteps = 1

    def _on_step(self) -> bool:
        """
        This method is called after each rollout is finished.
        """
        if self.num_timesteps == self.total_timesteps:
            return False

        self.num_timesteps += 1
        return True