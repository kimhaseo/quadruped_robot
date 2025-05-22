import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import config.config
from solver.trajectory import TrajectoryGenerator
from solver.inverse import Kinematics
from config.config import leg_resolution,hip_pose
from config.motor_cmd import AngleCommand
from controller.motor_controller import MotorController
from manager.pose_manager import pose_cmd
from solver.stabilizer import StabilizerSolver
import time
import numpy as np


class MotionController:
    def __init__(self):
        self.inverse_kinematics = Kinematics()
        self.motor_controller = MotorController()
        self.trajectory_generator = TrajectoryGenerator()
        self.pose_cmd = pose_cmd
        self.stabilizer = StabilizerSolver()

    def pose_control(self, target_pose, target_orientation, speed):

        current_pose = self.pose_cmd.get_pose()
        coords = self.trajectory_generator.generate_pose_trajectory(target_pose,current_pose)
        motor_speed  = speed * 40
        self.joint_control(coords,leg_resolution, 0.5, target_orientation, motor_speed)

    def move_control(self, speed, step_hight, distance, robot_motion,target_orientation):
        current_pose = self.pose_cmd.get_pose()
        coords = self.trajectory_generator.generate_move_trajectory(speed, step_hight, robot_motion)
        coords = list(coords)

        first_coords = coords[0][:int(leg_resolution/2)]
        second_coords = coords[0][int(leg_resolution/2):]

        zero_coord = np.array([[0, 0, 0] for _ in range(int(leg_resolution/2))], dtype=np.float32)

        # step_count, last_step_length = divmod(distance, speed)
        foot_coords = [0,0,0,0]


        foot_coords[0] = (np.array(current_pose['fl_foot']) + first_coords)
        foot_coords[1] = (np.array(current_pose['fr_foot']) + zero_coord)
        foot_coords[2] = (np.array(current_pose['rl_foot']) + zero_coord)
        foot_coords[3] = (np.array(current_pose['rr_foot']) + first_coords)

        self.joint_control(foot_coords, leg_resolution/2,0.5,target_orientation,4000)

        foot_coords[0] = (np.array(current_pose['fl_foot']) + second_coords)
        foot_coords[1] = (np.array(current_pose['fr_foot']) + first_coords)
        foot_coords[2] = (np.array(current_pose['rl_foot']) + first_coords)
        foot_coords[3] = (np.array(current_pose['rr_foot']) + second_coords)
        self.joint_control(foot_coords, leg_resolution/2,0.5,target_orientation,4000)

        foot_coords[0] = (np.array(current_pose['fl_foot']) + first_coords)
        foot_coords[1] = (np.array(current_pose['fr_foot']) + second_coords)
        foot_coords[2] = (np.array(current_pose['rl_foot']) + second_coords)
        foot_coords[3] = (np.array(current_pose['rr_foot']) + first_coords)
        self.joint_control(foot_coords, leg_resolution/2,0.5,target_orientation,4000)

        foot_coords[0] = (np.array(current_pose['fl_foot']) + second_coords)
        foot_coords[1] = (np.array(current_pose['fr_foot']) + zero_coord)
        foot_coords[2] = (np.array(current_pose['rl_foot']) + zero_coord)
        foot_coords[3] = (np.array(current_pose['rr_foot']) + second_coords)
        self.joint_control(foot_coords, leg_resolution/2,0.5,target_orientation,4000)


    def joint_control(self,coords,resolution,speed,target_orientation, motor_speed):

        diff_orientation = -(self.stabilizer.stabilize(target_orientation))
        delay = speed/resolution
        for i in range(int(resolution)):

            foot_coords = [coords[0][i],coords[1][i],coords[2][i],coords[3][i]]
            calibration_foot_coords = self.inverse_kinematics.calculate_foot_position_with_orientation(*diff_orientation,foot_coords)

            fl_foot = (np.array(calibration_foot_coords["fl_foot"]) - np.array(hip_pose["fl_hip"]))
            fr_foot = (np.array(calibration_foot_coords["fr_foot"]) - np.array(hip_pose["fr_hip"]))
            rl_foot = (np.array(calibration_foot_coords["rl_foot"]) - np.array(hip_pose["rl_hip"]))
            rr_foot = (np.array(calibration_foot_coords["rr_foot"]) - np.array(hip_pose["rr_hip"]))

            fl_degree1, fl_degree2, fl_degree3 = self.inverse_kinematics.calculate_joint_angle(False, *fl_foot)
            fr_degree1, fr_degree2, fr_degree3 = self.inverse_kinematics.calculate_joint_angle(True, *fr_foot)
            rl_degree1, rl_degree2, rl_degree3 = self.inverse_kinematics.calculate_joint_angle(False, *rl_foot)
            rr_degree1, rr_degree2, rr_degree3 = self.inverse_kinematics.calculate_joint_angle(True, *rr_foot)

            #
            angle_commands = [
                # AngleCommand("fl_joint1", -fl_degree1, motor_speed),
                AngleCommand("fl_joint2", -fl_degree2, motor_speed),
                AngleCommand("fl_joint3", -fl_degree3, motor_speed),
                # AngleCommand("fr_joint1", fr_degree1, motor_speed),
                AngleCommand("fr_joint2", fr_degree2, motor_speed),
                AngleCommand("fr_joint3", fr_degree3, motor_speed),
                # AngleCommand("rl_joint1", -rl_degree1, motor_speed),
                AngleCommand("rl_joint2", -rl_degree2, motor_speed),
                AngleCommand("rl_joint3", -rl_degree3, motor_speed),
                # AngleCommand("rr_joint1", rr_degree1, motor_speed),
                AngleCommand("rr_joint2", rr_degree2, motor_speed),
                AngleCommand("rr_joint3", rr_degree3, motor_speed),
            ]
            self.motor_controller.move_motors(angle_commands)
            time.sleep(delay)

            feet_positions = {
                "fl_foot": calibration_foot_coords["fl_foot"],
                "fr_foot": calibration_foot_coords["fr_foot"],
                "rl_foot": calibration_foot_coords["rl_foot"],
                "rr_foot": calibration_foot_coords["rr_foot"],
            }
            for foot, position in feet_positions.items():
                self.pose_cmd.update_pose(foot, position)
            print(pose_cmd.get_pose())

if __name__ == "__main__":
    controller = MotionController()

    target_pose = config.config.start_pose
    controller.pose_control(target_pose, [0, 0, 0], 20)
    time.sleep(2)
    target_pose = config.config.init_pose
    controller.pose_control(target_pose, [0, 0, 0],100)
    time.sleep(2)

    # for i in range(1):
    #     target_pose = config.config.init_pose
    #     controller.pose_control(target_pose, [0, 0, 0],1)
    #     print("init-pose")
    #     time.sleep(1)
    #     target_pose = config.config.down_pose
    #     controller.pose_control(target_pose, [0, 0, 0],1)
    #     print("down-pose")
    #     time.sleep(1)
    #     target_pose = config.config.init_pose
    #     controller.pose_control(target_pose, [0, 0, 0],1)   15
    #     print("init-pose")
    #     time.sleep(1)
    #
    # print("보행 시작")
    controller.move_control(40,40,40,"forward", [0,0,0])
    time.sleep(2)

    target_pose = config.config.start_pose
    controller.pose_control(target_pose, [0, 0, 0], 100)


