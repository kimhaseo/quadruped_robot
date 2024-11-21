import numpy as np
from config import config


step_hight = 20
resolution = config.leg_resolution
speed = 30 # mm/s
step_length = speed/2
hz= resolution * 2
motor_delay = 1/ hz


def foot_trajectory(time, is_left, step_length, step_hight):
    if is_left:
        x = np.where(
            (0 <= time) & (time < 2 / 2),
            step_length * (-1 + 2 * time / (2 / 2)),
            step_length * (1 - 2 * (time - 2 / 2) / (2 / 2))
        )
        z = np.where(
            (0 <= time) & (time < 2 / 2),
            step_hight * np.sin(np.pi * time / (2 / 2)),
            0
        )
    else:
        x = np.where(
            (0 <= time) & (time < 2 / 2),
            step_length * (1 - 2 * time / (2 / 2)),
            step_length * (-1 + 2 * (time - 2 / 2) / (2 / 2))
        )
        z = np.where(
            (0 <= time) & (time < 2 / 2),
            0,
            step_hight * np.sin(np.pi * (time - 2 / 2) / (2 / 2))
        )

    return np.array(x), np.array(z)


def generate_crawl_gait_pattern(resolution, hight=250):

    left_x, left_z = foot_trajectory(resolution, is_left=True)
    right_x, right_z = foot_trajectory(resolution, is_left=False)

    x_front_left = left_x
    x_front_right = right_x
    z_front_left = -hight + left_z
    z_front_right = -hight + right_z

    x_rear_left = x_front_right
    z_rear_left = z_front_right
    x_rear_right = x_front_left
    z_rear_right = z_front_left

    return (
        x_front_left, z_front_left,
        x_front_right, z_front_right,
        x_rear_left, z_rear_left,
        x_rear_right, z_rear_right
    )
