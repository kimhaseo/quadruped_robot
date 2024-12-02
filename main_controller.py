from controller.motion_controller import RobotController
from solver.trajectory import GaitPatternGenerator
from config.motor_cmd import MotorCommand, AngleCommand
from solver.inverse import Kinematics
from controller.motor_controller import MotorController
from config import config

def pose_control():

    fl_trajectory = gpg.generate_point_trajectory(config.init_pose["fl_coord"], config.start_pose["fl_coord"])
    fr_trajectory = gpg.generate_point_trajectory(config.init_pose["fr_coord"], config.start_pose["fr_coord"])
    rl_trajectory = gpg.generate_point_trajectory(config.init_pose["rl_coord"], config.start_pose["rl_coord"])
    rr_trajectory = gpg.generate_point_trajectory(config.init_pose["rr_coord"], config.start_pose["rr_coord"])

    for i in range(200):

        fl_theta1,fl_theta2,fl_theta3 = ik.calculate_joint_angle(False,fl_trajectory[i][0],fl_trajectory[i][1],fl_trajectory[i][2])
        fr_theta1,fr_theta2,fr_theta3 = ik.calculate_joint_angle(False, fr_trajectory[i][0], fr_trajectory[i][1],fr_trajectory[i][2])
        rl_theta1,rl_theta2,rl_theta3 = ik.calculate_joint_angle(False, rl_trajectory[i][0], rl_trajectory[i][1],rl_trajectory[i][2])
        rr_theta1,rr_theta2,rr_theta3 = ik.calculate_joint_angle(False, rr_trajectory[i][0], rr_trajectory[i][1],rr_trajectory[i][2])

        angle_commands = [
            AngleCommand("fl_joint1", -fl_theta1),
            AngleCommand("fl_joint2", -fl_theta2),
            AngleCommand("fl_joint3", -2 * fl_theta3),
            AngleCommand("fr_joint1", fr_theta1),  # 괄호 수정됨
            AngleCommand("fr_joint2", fr_theta2),
            AngleCommand("fr_joint3", 2 * fr_theta3),
            AngleCommand("rl_joint1", -rl_theta1),
            AngleCommand("rl_joint2", -rl_theta2),
            AngleCommand("rl_joint3", -2 * rl_theta3),
            AngleCommand("rr_joint1", rr_theta1),
            AngleCommand("rr_joint2", rr_theta2),
            AngleCommand("rr_joint3", 2 * rr_theta3)
        ]
        mc.move_motors(angle_commands)

## 주행 , 포즈 변경(타겟 포즈) ,

def main():
    init_robot()
    rc.drive(robot_speed=120, distance=2400, step_height=50, motion="forward", orientation=[0, 0, 0])



if __name__ == "__main__":
    main()

    rc = RobotController()
    gpg = GaitPatternGenerator()
    m_cmd = MotorCommand
    ik = Kinematics()
    mc= MotorController()


