import time
from manager.pose_manager import pose_cmd
import math
# 주어진 값


robot_vel = [10, 0]  # 속도 벡터 [vx, vy] cm/s
# current_position = [0, 0, 0]  # 초기 위치 [x, y, theta]
# 시간 간격 (1초)
delta_t = 0.01

while True :
    current_pose = pose_cmd.get_pose()
    current_yaw = current_pose['body_orientation'][2]
    current_position = current_pose['body_position']
    # 속도에 따라 새로운 위치 계산
    print("robot_yaw: ",current_yaw)
    cos_theta = math.cos(current_yaw)
    sin_theta = math.sin(current_yaw)

    # 속도 벡터를 회전시킴
    rotated_vel_x = robot_vel[0] * cos_theta - robot_vel[1] * sin_theta
    rotated_vel_y = robot_vel[0] * sin_theta + robot_vel[1] * cos_theta

    # 새 위치 계산
    current_position[0] = current_position[0] + rotated_vel_x * delta_t
    current_position[1] = current_position[1] + rotated_vel_y * delta_t
    current_position[2] = (current_pose['fl_foot'][2]+current_pose['fr_foot'][2]+current_pose['rl_foot'][2]+current_pose['rr_foot'][2]) / 4
    time.sleep(delta_t)
    pose_cmd.update_pose("body_position", current_position)
    print(current_position)


