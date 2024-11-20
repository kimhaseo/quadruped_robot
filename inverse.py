import numpy as np
import math
import config

class LegKinematics:

    def __init__(self):
        # 다리 길이를 설정에서 가져옴
        self.L1 = config.leg_length["L1"]
        self.L2 = config.leg_length["L2"]
        self.L3 = config.leg_length["L3"]
        self.body_length = config.body_dimensions["length"]
        self.body_width = config.body_dimensions["width"]
        self.body_height = config.body_dimensions["height"]

        self.hip_positions = np.array([
            [self.body_length / 2, -self.body_width / 2, 0],  # 앞 왼쪽
            [self.body_length / 2, +self.body_width / 2, 0],  # 앞 오른쪽
            [-self.body_length / 2, -self.body_width / 2, 0],  # 뒤 왼쪽
            [-self.body_length / 2, +self.body_width / 2, 0],  # 뒤 오른쪽
        ])



    def calculate_leg_position(self, right_leg, x, y, z):

        A = np.sqrt(y ** 2 + z ** 2)
        a1 = np.degrees(math.atan2(y, z))
        a2 = np.degrees(np.arccos(self.L1 / A))
        if right_leg:
            theta1 = 90 - (a1 - a2)
            z_x = -y * math.sin(math.radians(-theta1)) + z * math.cos(math.radians(theta1))
        else:
            theta1 = 90 + (a1 + a2)
            z_x = -y * math.sin(math.radians(theta1)) + z * math.cos(math.radians(-theta1))
        theta1 = round(theta1, 2)
        z_x = round(z_x, 2)

        b1 = np.degrees(math.atan2(z_x, x))
        B = np.sqrt(x ** 2 + z_x ** 2)
        if b1 < 180:
            b1 += 360
        b2 = np.degrees(np.arccos((self.L2 ** 2 + B ** 2 - self.L3 ** 2) / (2 * self.L2 * B)))
        theta2 = b1 - b2 - 180
        theta2 = round(theta2, 2)

        b3 = np.degrees(np.arccos((self.L2 ** 2 + self.L3 ** 2 - B ** 2) / (2 * self.L2 * self.L3)))
        theta3 = 180 - b3
        theta3 = round(theta3, 2)

        return theta1, theta2, theta3

