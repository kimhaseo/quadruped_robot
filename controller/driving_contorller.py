from solver.trajectory import GaitPatternGenerator
from solver.inverse import  Kinematics
from config.motor_cmd import AngleCommand
from controller.motor_controller import MotorController
from config.config import leg_resolution
import time
import math

robot_speed = 30 #mm/s
distance = 120 #mm
robot_step_hight = 30 #mm
robot_motion = "forward"
robot_orientaion = [0, 0, 0]
motor_delay = 1 / leg_resolution

inverse_kinematics = Kinematics()
motor_controller = MotorController()

def driving_contoller():

    pattern_generator = GaitPatternGenerator()

    step_count = distance / robot_speed
    print(step_count)
    if math.isclose(step_count, round(step_count)):  # 부동소수점 정밀도 해결
        step_count = round(step_count)
        foot_poses = pattern_generator.generate_crawl_gait_pattern(robot_speed, robot_step_hight, robot_motion)
        motor_control(foot_poses, step_count)

    elif step_count ==0:
        foot_poses = pattern_generator.generate_crawl_gait_pattern(robot_speed, robot_step_hight, robot_motion)
        motor_control(foot_poses, 1000)

    else:
        # 정수가 아닌 경우 처리
        step_count_int = math.floor(step_count)
        foot_poses = pattern_generator.generate_crawl_gait_pattern(robot_speed, robot_step_hight, robot_motion)
        motor_control(foot_poses, step_count_int)

        remaining_distance = distance - (int(step_count) * robot_speed)
        adjusted_speed = remaining_distance   # 나머지 거리만큼 속도 조정
        foot_poses = pattern_generator.generate_crawl_gait_pattern(adjusted_speed, robot_step_hight, robot_motion)
        motor_control(foot_poses, 1)

def motor_control(foot_pose,step_count):

    base_foot_poses = inverse_kinematics.calculate_foot_position_with_orientation(*robot_orientaion)
    num_steps = leg_resolution

    for i in range(step_count):
        for i in range(num_steps):

            start_time = time.perf_counter()

            fl_theta1, fl_theta2, fl_theta3 = inverse_kinematics.calculate_joint_angle(False, foot_pose[0][0][i]+base_foot_poses[0][0], foot_pose[0][1][i]+base_foot_poses[0][1], foot_pose[0][2][i]+base_foot_poses[0][2])
            fr_theta1, fr_theta2, fr_theta3 = inverse_kinematics.calculate_joint_angle(True, foot_pose[1][0][i]+base_foot_poses[1][0], foot_pose[1][1][i]+base_foot_poses[1][1], foot_pose[1][2][i]+base_foot_poses[1][2])
            rl_theta1, rl_theta2, rl_theta3 = inverse_kinematics.calculate_joint_angle(False, foot_pose[2][0][i]+base_foot_poses[2][0], foot_pose[2][1][i]+base_foot_poses[2][1], foot_pose[2][2][i]+base_foot_poses[2][2])
            rr_theta1, rr_theta2, rr_theta3 = inverse_kinematics.calculate_joint_angle(True, foot_pose[3][0][i]+base_foot_poses[3][0], foot_pose[3][1][i]+base_foot_poses[3][1], foot_pose[3][2][i]+base_foot_poses[3][2])

            angle_commands = [
                AngleCommand("fl_joint1", -fl_theta1),
                AngleCommand("fl_joint2", -fl_theta2),
                AngleCommand("fl_joint3", -2 * (fl_theta3)),
                AngleCommand("fr_joint1", fr_theta1),
                AngleCommand("fr_joint2", fr_theta2),
                AngleCommand("fr_joint3", 2 * (fr_theta3)),
                AngleCommand("rl_joint1", rl_theta1),
                AngleCommand("rl_joint2", rl_theta2),
                AngleCommand("rl_joint3", 2 * (rl_theta3)),
                AngleCommand("rr_joint1", -rr_theta1),
                AngleCommand("rr_joint2", -rr_theta2),
                AngleCommand("rr_joint3", -2 *(rr_theta3)),
            ]
#
#             # 모터 명령 동기적으로 실행
            motor_controller.move_motors(angle_commands)
#             # 약간의 지연 추가
            while time.perf_counter() - start_time < motor_delay:
                pass  # 원하는 시간만큼 대기

    motor_controller.close()

if __name__ == "__main__":
    driving_contoller()