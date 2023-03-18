import numpy as np
from gym import spaces


class ObservationManager:
    def __init__(self, env):
        self.env = env

    def encode_observation(self, observation):
        return np.array([
            observation["robot"]["position"]["x"],
            observation["robot"]["position"]["y"],
            observation["robot"]["position"]["theta"],
            observation["robot"]["twist"]["x"],
            observation["robot"]["twist"]["y"],
            observation["robot"]["twist"]["theta"],
            observation["target"]["position"]["x"],
            observation["target"]["position"]["y"],
        ], dtype=np.float32).flatten()

    def get_observation(self):
        robot_odometry = self.env.robot.get_odometry()
        target_odometry = self.env.target.get_odometry()
        return {
            "robot": {
                "position": {
                    "x": robot_odometry.pose.pose.position.x,
                    "y": robot_odometry.pose.pose.position.y,
                    "theta": robot_odometry.pose.pose.orientation.z
                },
                "twist": {
                    "x": robot_odometry.twist.twist.linear.x,
                    "y": robot_odometry.twist.twist.linear.y,
                    "theta": robot_odometry.twist.twist.angular.z
                }
            },
            "target": {
                "position": {
                    "x": target_odometry.pose.pose.position.x,
                    "y": target_odometry.pose.pose.position.y,
                    "theta": target_odometry.pose.pose.orientation.z
                },
            }
        }

    def create_observation_space(
            self,
            robot_position_x,
            robot_position_y,
            robot_orientation,
            robot_linear_velocity_x,
            robot_linear_velocity_y,
            robot_angular_velocity,
            target_position_x,
            target_position_y):

        return spaces.Box(
            low=np.array([
                robot_position_x[0],
                robot_position_y[0],
                robot_orientation[0],
                robot_linear_velocity_x[0],
                robot_linear_velocity_y[0],
                robot_angular_velocity[0],
                target_position_x[0],
                target_position_y[0],
            ]).flatten(),
            high=np.array([
                robot_position_x[1],
                robot_position_y[1],
                robot_orientation[1],
                robot_linear_velocity_x[1],
                robot_linear_velocity_y[1],
                robot_angular_velocity[1],
                target_position_x[1],
                target_position_y[1],
            ]).flatten(),
            dtype=np.float32
        )


class ActionDecoder:
    def __call__(self, encoded_action: np.array):
        return {"linear": encoded_action[0], "angular": encoded_action[1]}


def create_action_space(
        robot_linear_velocity_internal,
        robot_angular_velocity_internal):
    return spaces.Box(
        low=np.array([
            robot_linear_velocity_internal[0],
            robot_angular_velocity_internal[0]
        ]),
        high=np.array([
            robot_linear_velocity_internal[1],
            robot_angular_velocity_internal[1]
        ]),
        dtype=np.float32
    )
