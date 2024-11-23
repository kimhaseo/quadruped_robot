import numpy as np
from config import config


class GaitPatternGenerator:
    def __init__(self):
        """
        Gait Pattern Generator 초기화.

        :param step_height: 발의 높이 (mm)
        :param speed: 이동 속도 (mm/s)
        :param height: 기본 발 위치 높이 (mm)
        """
        self.resolution = config.leg_resolution
        # self.hz = self.resolution * 2
        # self.motor_delay = 1 / self.hz


    def foot_trajectory(self, time, start_step, speed, step_height):
        """
        한쪽 발의 궤적 계산.
        :param time: 시간 배열
        :param is_left: 왼쪽 발인지 여부 (True/False)
        :return: x, z 좌표 배열
        """

        self.step_length = speed / 2
        self.step_height = step_height

        if start_step == "forward" :
            x = np.where(
                (0 <= time) & (time < 2 / 2),
                self.step_length * (-1 + 2 * time / (2 / 2)),
                self.step_length * (1 - 2 * (time - 2 / 2) / (2 / 2))
            )
            y = np.zeros_like(time)
            z = np.where(
                (0 <= time) & (time < 2 / 2),
                self.step_height * np.sin(np.pi * time / (2 / 2)),
                0
            )
        elif start_step == "backward" :
            x = np.where(
                (0 <= time) & (time < 2 / 2),
                self.step_length * (1 - 2 * time / (2 / 2)),
                self.step_length * (-1 + 2 * (time - 2 / 2) / (2 / 2))
            )
            y = np.zeros_like(time)
            z = np.where(
                (0 <= time) & (time < 2 / 2),
                0,
                self.step_height * np.sin(np.pi * (time - 2 / 2) / (2 / 2))
            )

        elif start_step == "left" :

            pass

        elif start_step == "right" :

            pass

        return np.array(x), np.array(y) , np.array(z)

    def generate_crawl_gait_pattern(self, speed, step_hight, robot_motion):

        self.speed = speed
        self.step_hight = step_hight
        self.robot_motion = robot_motion
        time = np.linspace(0, 2, self.resolution)

        if robot_motion == "forward":
            foot_direction = ["forward","backward","backward","forward"]

        fl_x, fl_y, fl_z = self.foot_trajectory(time, foot_direction[0], self.speed , self.step_hight)
        fr_x, fr_y, fr_z = self.foot_trajectory(time, foot_direction[1], self.speed , self.step_hight)
        rl_x, rl_y, rl_z = self.foot_trajectory(time, foot_direction[2], self.speed , self.step_hight)
        rr_x, rr_y, rr_z = self.foot_trajectory(time, foot_direction[3], self.speed , self.step_hight)


        #
        #
        # return (
        #     x_front_left, z_front_left,
        #     x_front_right, z_front_right,
        #     x_rear_left, z_rear_left,
        #     x_rear_right, z_rear_right
        # )

if __name__ == "__main__":

    gpg=GaitPatternGenerator()
    trajetory = gpg.generate_crawl_gait_pattern(10,10,"forward")
