
import numpy as np
import math
from config import config

class Kinematics:

    def __init__(self):
        # 다리 길이를 설정에서 가져옴
        self.L1 = config.leg_length["L1"]
        self.L2 = config.leg_length["L2"]
        self.L3 = config.leg_length["L3"]
        self.body_length = config.body_dimensions["length"]
        self.body_width = config.body_dimensions["width"]
        self.body_height = config.body_dimensions["height"]

        self.hip_positions = np.array([
            [+self.body_length / 2, -self.body_width / 2, 0],  # 앞 왼쪽
            [+self.body_length / 2, +self.body_width / 2, 0],  # 앞 오른쪽
            [-self.body_length / 2, -self.body_width / 2, 0],  # 뒤 왼쪽
            [-self.body_length / 2, +self.body_width / 2, 0],  # 뒤 오른쪽
        ])

        self.foot_positions = np.array([
            [+self.body_length / 2, -self.body_width / 2 - self.L1, -self.body_height],  # 앞 왼쪽
            [+self.body_length / 2, +self.body_width / 2 + self.L1, -self.body_height],  # 앞 오른쪽
            [-self.body_length / 2, -self.body_width / 2 - self.L1, -self.body_height],  # 뒤 왼쪽
            [-self.body_length / 2, +self.body_width / 2 + self.L1, -self.body_height],  # 뒤 오른쪽
        ])

    def calculate_foot_position_with_orientation(self, roll, pitch, yaw):
        """
        Roll, Pitch, Yaw를 적용하여 발 위치를 계산.

        Parameters:
        roll: float, x축에 대한 회전 (단위: degree)
        pitch: float, y축에 대한 회전 (단위: degree)
        yaw: float, z축에 대한 회전 (단위: degree)

        Returns:
        np.array: 각 발의 새로운 3D 위치 배열 (앞에 True/False 추가)
        """

        # 도를 라디안으로 변환
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)

        # Roll, Pitch, Yaw 회전 행렬 계산
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(roll_rad), -math.sin(roll_rad)],
            [0, math.sin(roll_rad), math.cos(roll_rad)]
        ])

        R_y = np.array([
            [math.cos(pitch_rad), 0, math.sin(pitch_rad)],
            [0, 1, 0],
            [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]
        ])

        R_z = np.array([
            [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
            [math.sin(yaw_rad), math.cos(yaw_rad), 0],
            [0, 0, 1]
        ])

        # 전체 회전 행렬
        R = R_z @ R_y @ R_x

        # 초기 발 위치에 회전 행렬 적용
        rotated_positions = np.dot(self.foot_positions, R.T)
        calc_foot_pose = rotated_positions - self.hip_positions
        return calc_foot_pose

    def calculate_joint_angle(self, right_leg, x, y, z):
        try:
            A = np.sqrt(y ** 2 + z ** 2)
            a1 = np.degrees(math.atan2(y, z))
            a2 = np.degrees(np.arccos(self.L1 / A))  # self.L1로 수정

            if right_leg == True:
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

            b2 = np.degrees(np.arccos((self.L2 ** 2 + B ** 2 - self.L3 ** 2) / (2 * self.L2 * B)))  # self.L2, self.L3로 수정
            theta2 = b1 - b2 - 180
            theta2 = round(theta2, 2)

            b3 = np.degrees(np.arccos((self.L2 ** 2 + self.L3 ** 2 - B ** 2) / (2 * self.L2 * self.L3)))  # self.L2, self.L3로 수정
            theta3 = 180 - b3
            theta3 = round(theta3, 2)

            # 범위 체크
            if abs(theta1) > 20 or abs(theta2) > 90 or abs(theta3) > 160:
                raise ValueError(f"Invalid joint angles: theta1={theta1}, theta2={theta2}, theta3={theta3}")

            return theta1, theta2, theta3

        except ValueError as e:
            print(f"Error: {e}")
            return None

if __name__ == "__main__":
    # 테스트 입력 (roll, pitch, yaw 값은 도 단위)
    roll = 0  # x축 회전 (degrees)
    pitch = 10  # y축 회전 (degrees)
    yaw = 0  # z축 회전 (degrees)

    kinematics = Kinematics()
    # new_foot_positions = (kinematics.calculate_foot_position_with_orientation(roll, pitch, yaw) - kinematics.hip_positions)
    new_foot_positions = (kinematics.calculate_foot_position_with_orientation(roll, pitch, yaw))

    print(new_foot_positions)


    # for i, foot_position in enumerate(new_foot_positions):
    #     if i % 2 == 0:  # 오른쪽 다리
    #         right_leg = True
    #     else:  # 왼쪽 다리
    #         right_leg = False
    #
    #     # foot_position에서 x, y, z 분리
    #     x, y, z = foot_position
    #
    #     print(f"다리 {i + 1}의 발 위치 (x, y, z): {foot_position}")
    #
    #     # calculate_joint_angle 함수에 x, y, z를 개별 인자로 전달
    #     joint_angles = kinematics.calculate_joint_angle(right_leg, x, y, z)
    #     if joint_angles:
    #         print(f"다리 {i + 1} 관절 각도 (theta1, theta2, theta3): {joint_angles}")
    #     else:
    #         print(f"다리 {i + 1}의 관절 각도 계산 오류")


