from solver.trajectory import GaitPatternGenerator
from solver.inverse import  Kinematics
from config.config import leg_resolution
#from controller.motor_controller import MotorController

robot_speed = 30 #mm/s
robot_step_hight = 30
robot_motion = "forward"
robot_oriention = [0, 0, 0]

inverse_kinematics = Kinematics()

def driving_contoller():

    pattern_generator = GaitPatternGenerator()
    inverse_kinematics = Kinematics()
    foot_poses = pattern_generator.generate_crawl_gait_pattern(robot_speed,robot_step_hight,robot_motion)
    motor_control(foot_poses)

def motor_control(foot_pose):

    base_foot_poses = inverse_kinematics.calculate_foot_position_with_orientation(*robot_oriention)
    num_steps = leg_resolution
    print(foot_pose[0][1][0]+base_foot_poses[0][1])
    for i in range(10):
        for i in range(num_steps):
            fl_theta1, fl_theta2, fl_theta3 = inverse_kinematics.calculate_joint_angle(False, foot_pose[0][0][i]+base_foot_poses[0][0], foot_pose[0][1][i]+base_foot_poses[0][1], foot_pose[0][2][i]+base_foot_poses[0][2])
            fr_theta1, fr_theta2, fr_theta3 = inverse_kinematics.calculate_joint_angle(True, foot_pose[1][0][i]+base_foot_poses[1][0], foot_pose[1][1][i]+base_foot_poses[1][1], foot_pose[1][2][i]+base_foot_poses[1][2])
            rl_theta1, rl_theta2, rl_theta3 = inverse_kinematics.calculate_joint_angle(False, foot_pose[2][0][i]+base_foot_poses[2][0], foot_pose[2][1][i]+base_foot_poses[2][1], foot_pose[2][2][i]+base_foot_poses[2][2])
            rr_theta1, rr_theta2, rr_theta3 = inverse_kinematics.calculate_joint_angle(True, foot_pose[3][0][i]+base_foot_poses[3][0], foot_pose[3][1][i]+base_foot_poses[3][1], foot_pose[3][2][i]+base_foot_poses[3][3])

            print(fl_theta1)
#             angle_commands = [
#                 AngleCommand("lf_joint1", -fl_theta2),
#                 # AngleCommand("lf_joint2", -fl_theta2),
#                 # AngleCommand("lf_joint3", -2 * (fl_theta3)),
#                 # AngleCommand("rf_joint1", -fr_theta1),
#                 AngleCommand("rf_joint2", -2 * (fl_theta3)),
#                 # AngleCommand("rf_joint3", -2 * (fr_theta3)),
#                 # AngleCommand("lr_joint1", -rl_theta1),
#                 # AngleCommand("lr_joint2", -rl_theta2),
#                 # AngleCommand("lr_joint3", -2 * (rl_theta3)),
#                 # AngleCommand("rr_joint1", -rr_theta1),
#                 # AngleCommand("rr_joint2", -rr_theta2),
#                 # AngleCommand("rr_joint3", -2 *(rr_theta3)),
#             ]
#
#             # 모터 명령 동기적으로 실행
#             motor_controller.move_motors(angle_commands)
#             # 약간의 지연 추가
#             time.sleep(motor_delay)
#     motor_controller.close()

if __name__ == "__main__":
    driving_contoller()