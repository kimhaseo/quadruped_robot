import numpy as np
from config import config


class GaitPatternGenerator:
    def __init__(self, step_height=20, speed=30, height=250):
        """
        Gait Pattern Generator 초기화.

        :param step_height: 발의 높이 (mm)
        :param speed: 이동 속도 (mm/s)
        :param height: 기본 발 위치 높이 (mm)
        """
        self.step_height = step_height
        self.speed = speed
        self.step_length = speed / 2
        self.resolution = config.leg_resolution
        self.hz = self.resolution * 2
        self.motor_delay = 1 / self.hz
        self.height = height

    def foot_trajectory(self, time, is_left):
        """
        한쪽 발의 궤적 계산.

        :param time: 시간 배열
        :param is_left: 왼쪽 발인지 여부 (True/False)
        :return: x, z 좌표 배열
        """
        step_length = self.step_length
        step_height = self.step_height

        if is_left:
            x = np.where(
                (0 <= time) & (time < 2 / 2),
                step_length * (-1 + 2 * time / (2 / 2)),
                step_length * (1 - 2 * (time - 2 / 2) / (2 / 2))
            )
            z = np.where(
                (0 <= time) & (time < 2 / 2),
                step_height * np.sin(np.pi * time / (2 / 2)),
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
                step_height * np.sin(np.pi * (time - 2 / 2) / (2 / 2))
            )

        return np.array(x), np.array(z)

    def generate_crawl_gait_pattern(self):
        """
        크롤 게이트 패턴 생성.

        :return: 각 다리의 x, z 좌표 배열
        """
        time = np.linspace(0, 2, self.resolution)

        left_x, left_z = self.foot_trajectory(time, is_left=True)
        right_x, right_z = self.foot_trajectory(time, is_left=False)

        x_front_left = left_x
        x_front_right = right_x
        z_front_left = -self.height + left_z
        z_front_right = -self.height + right_z

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