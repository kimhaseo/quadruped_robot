from motor_controller import MotorController
from motor_cmd import MotorCommand
import numpy as np
from datetime import datetime
import math
import time
# 다리의 길이
L1 = 50  # mm
L2 = 200
L3 = 200

# 로봇 몸체에서 각 다리의 원점 좌표 설정 (앞-뒤, 좌-우 대칭)
hip_positions = {
    "front_left": np.array([300, -50, 0]),  # 앞 왼쪽 다리
    "front_right": np.array([300, 50, 0]),  # 앞 오른쪽 다리
    "rear_left": np.array([-300, -50, 0]),  # 뒤 왼쪽 다리
    "rear_right": np.array([-300, +50, 0])  # 뒤 오른쪽 다리
}


# 다리 위치 계산 함수 (중복된 함수 정의 제거)
def calculate_leg_position(right_leg, x, y, z, hip_pos):
    A = np.sqrt(y ** 2 + z ** 2)
    a1 = np.degrees(math.atan2(y, z))
    a2 = np.degrees(np.arccos(L1 / A))
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
    b2 = np.degrees(np.arccos((L2 ** 2 + B ** 2 - L3 ** 2) / (2 * L2 * B)))
    theta2 = b1 - b2 - 180
    theta2 = round(theta2, 2)

    b3 = np.degrees(np.arccos((L2 ** 2 + L3 ** 2 - B ** 2) / (2 * L2 * L3)))
    theta3 = 180 - b3
    theta3 = round(theta3, 2)
    b4 = 90 - b3 + theta2

    return theta1, theta2, theta3


# 발의 궤적을 생성하는 함수
def foot_trajectory(time, is_left):
    if is_left:
        x = np.where(
            (0 <= time) & (time < 2 / 2),
            80 * (-1 + 2 * time / (2 / 2)),
            80 * (1 - 2 * (time - 2 / 2) / (2 / 2))
        )
        z = np.where(
            (0 <= time) & (time < 2 / 2),
            70 * np.sin(np.pi * time / (2 / 2)),
            0
        )
    else:
        x = np.where(
            (0 <= time) & (time < 2 / 2),
            80 * (1 - 2 * time / (2 / 2)),
            80 * (-1 + 2 * (time - 2 / 2) / (2 / 2))
        )
        z = np.where(
            (0 <= time) & (time < 2 / 2),
            0,
            70 * np.sin(np.pi * (time - 2 / 2) / (2 / 2))
        )

    return np.array(x), np.array(z)


# 크롤 패턴 생성 함수
def generate_crawl_gait_pattern():
    T = 2
    dt = 0.01
    time = np.arange(0, T, dt)
    left_x, left_z = foot_trajectory(time, is_left=True)
    right_x, right_z = foot_trajectory(time, is_left=False)

    x_front_left = left_x
    x_front_right = right_x
    z_front_left = -300 + left_z
    z_front_right = -300 + right_z

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


# 모터 제어 함수 (비동기 처리 방식 수정)
def motor_control(x_range, z_range):
    num_steps = len(x_range[0])
    motor_controller = MotorController()  # MotorController 초기화
    while True:
        for i in range(num_steps):
            fl_theta1, fl_theta2, fl_theta3 = calculate_leg_position(False, x_range[0][i], -50, z_range[0][i],
                                                                     hip_positions["front_left"])
            fr_theta1, fr_theta2, fr_theta3 = calculate_leg_position(True, x_range[1][i], 50, z_range[1][i],
                                                                     hip_positions["front_right"])
            rl_theta1, rl_theta2, rl_theta3 = calculate_leg_position(False, x_range[2][i], -50, z_range[2][i],
                                                                     hip_positions["rear_left"])
            rr_theta1, rr_theta2, rr_theta3 = calculate_leg_position(True, x_range[3][i], 50, z_range[3][i],
                                                                     hip_positions["rear_right"])

            motor_commands = [
                MotorCommand("lf_joint1", -fl_theta2),
                MotorCommand("rf_joint2", -2 * (fl_theta3)),
            ]

            # 모터 명령 동기적으로 실행
            motor_controller.move_motors(motor_commands)

            # 약간의 지연 추가
            time.sleep(0.015)
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 마이크로초 단위 시간 포함
            print(f"현재 시간: {current_time}")


def main():
    # 발의 궤적 생성
    x_front_left, z_front_left, x_front_right, z_front_right, x_rear_left, z_rear_left, x_rear_right, z_rear_right = generate_crawl_gait_pattern()

    # 모터 제어 실행
    motor_control(
        [x_front_left, x_front_right, x_rear_left, x_rear_right],
        [z_front_left, z_front_right, z_rear_left, z_rear_right]
    )


if __name__ == "__main__":
    main()