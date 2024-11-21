import numpy as np
from config import config

# 설정
step_height = 20
body_high = 250
resolution = np.linspace(0, 1, config.leg_resolution)  # 배열로 정의
speed = 30  # mm/s
step_length = speed / 2

# 방향 상수
forward = "forward"
backward = "backward"
right = "right"
left = "left"


def foot_trajectory(direction):
    """다리의 궤적 생성"""

    if direction == forward:
        x = np.where(
            (0 <= resolution) & (resolution < 0.5),
            step_length * (-1 + 2 * resolution / 0.5) + step_length,
            step_length * (1 - 2 * (resolution - 0.5) / 0.5 ) + step_length
        )
        z = np.where(
            (0 <= resolution) & (resolution < 0.5),
            step_height * np.sin(np.pi * resolution / 0.5) - body_high,
            0
        )
    elif direction == backward:
        x = np.where(
            (0 <= resolution) & (resolution < 0.5),
            step_length * (1 - 2 * resolution / 0.5)+ step_length,
            step_length * (-1 + 2 * (resolution - 0.5) / 0.5) + step_length
        )
        z = np.where(
            (0 <= resolution) & (resolution < 0.5),
            0,
            step_height * np.sin(np.pi * (resolution - 0.5) / 0.5) - body_high
        )
    return np.array(x), np.array(z)


def generate_crawl_gait_pattern(body_direction):
    """크롤링 보행 패턴 생성"""

    def adjust_leg_trajectory(start_step, flip_x=False):
        x, z = foot_trajectory(start_step)
        if flip_x:
            x = -x
        return x, z

    # 방향에 따른 다리 궤적 설정
    if body_direction == forward:
        fl = adjust_leg_trajectory(forward)
        fr = adjust_leg_trajectory(backward)
        rl = adjust_leg_trajectory(backward)
        rr = adjust_leg_trajectory(forward)
    elif body_direction == backward:
        fl = adjust_leg_trajectory(forward, flip_x=True)
        fr = adjust_leg_trajectory(backward, flip_x=True)
        rl = adjust_leg_trajectory(backward, flip_x=True)
        rr = adjust_leg_trajectory(forward, flip_x=True)
    elif body_direction == right:
        fl = adjust_leg_trajectory(forward)
        fr = adjust_leg_trajectory(backward, flip_x=True)
        rl = adjust_leg_trajectory(forward)
        rr = adjust_leg_trajectory(backward, flip_x=True)
    elif body_direction == left:
        fl = adjust_leg_trajectory(backward, flip_x=True)
        fr = adjust_leg_trajectory(forward)
        rl = adjust_leg_trajectory(backward, flip_x=True)
        rr = adjust_leg_trajectory(forward)

    # 모든 다리의 궤적 반환

    return fl + fr + rl + rr

if __name__ == '__main__':
    generate_crawl_gait_pattern(backward)