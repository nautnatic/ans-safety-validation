import random
import math
from geometry_msgs.msg import Pose2D


class RandomBoxPoseGenerator:
    """
    Generates a random pose within the environment defined by a box
    """

    def __init__(self, x_range, y_range, theta_range=[-math.pi, math.pi]):
        self.x_range = x_range
        self.y_range = y_range
        self.theta_range = theta_range

    def __call__(self):
        return Pose2D(
            x=random.uniform(self.x_range[0], self.x_range[1]),
            y=random.uniform(self.y_range[0], self.y_range[1]),
            theta=random.uniform(self.theta_range[0], self.theta_range[1])
        )
