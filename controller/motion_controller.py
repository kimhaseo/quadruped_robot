import sys
import os

from keyboard.mouse import click

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
        # self.motor_controller = MotorController()
        self.trajectory_generator = TrajectoryGenerator()
        self.pose_cmd = pose_cmd
        self.stabilizer = StabilizerSolver()

    def pose_control(self, target_pose, target_orientation, speed):

        current_pose = self.pose_cmd.get_pose()
        coords = self.trajectory_generator.generate_pose_trajectory(target_pose,current_pose)
        motor_speed  = speed * 40
        self.joint_control(coords,leg_resolution, 0.5, target_orientation, motor_speed)

    def move_control(self, speed, step_height, distance, robot_motion,target_orientation):

        self.pose_control(config.config.init_pose, [0, 0, 0], 100)
        current_pose = self.pose_cmd.get_pose()
        coords = self.trajectory_generator.generate_move_trajectory(speed, step_height, robot_motion)
        coords = list(coords)

        first_coords = coords[0][:int(leg_resolution/2)]
        second_coords = coords[0][int(leg_resolution/2):]

        zero_coord = np.array([[0, 0, 0] for _ in range(int(leg_resolution/2))], dtype=np.float32)

        step_count, last_step_length = divmod(distance, speed)
        foot_coords = [0,0,0,0]

        foot_coords[0] = (np.array(current_pose['fl_foot']) + first_coords)
        foot_coords[1] = (np.array(current_pose['fr_foot']) + zero_coord)
        foot_coords[2] = (np.array(current_pose['rl_foot']) + zero_coord)
        foot_coords[3] = (np.array(current_pose['rr_foot']) + first_coords)

        self.joint_control(foot_coords, leg_resolution/2,0.125,target_orientation,4000)

        for i in range(step_count):

            step_count = step_count - 1

            foot_coords[0] = (np.array(current_pose['fl_foot']) + second_coords)
            foot_coords[1] = (np.array(current_pose['fr_foot']) + first_coords)
            foot_coords[2] = (np.array(current_pose['rl_foot']) + first_coords)
            foot_coords[3] = (np.array(current_pose['rr_foot']) + second_coords)
            self.joint_control(foot_coords, leg_resolution/2,0.125,target_orientation,4000)

            if step_count == 0:
                foot_coords[0] = (np.array(current_pose['fl_foot']) + zero_coord)
                foot_coords[1] = (np.array(current_pose['fr_foot']) + second_coords)
                foot_coords[2] = (np.array(current_pose['rl_foot']) + second_coords)
                foot_coords[3] = (np.array(current_pose['rr_foot']) + zero_coord)
                self.joint_control(foot_coords, leg_resolution / 2, 0.125, target_orientation, 4000)
                break

            else:
                foot_coords[0] = (np.array(current_pose['fl_foot']) + first_coords)
                foot_coords[1] = (np.array(current_pose['fr_foot']) + second_coords)
                foot_coords[2] = (np.array(current_pose['rl_foot']) + second_coords)
                foot_coords[3] = (np.array(current_pose['rr_foot']) + first_coords)
                self.joint_control(foot_coords, leg_resolution/2,0.125,target_orientation,4000)

    def joint_control(self, coords, resolution, speed, target_orientation, motor_speed):
        diff_orientation = -(self.stabilizer.stabilize(target_orientation))

        # 보정된 좌표를 누적할 리스트 (for 루프 바깥에서 선언)
        calibrated_coords_list = [[], [], [], []]

        for i in range(int(resolution)):
            foot_coords_fl = coords[0][i]
            foot_coords_fr = coords[1][i]
            foot_coords_rl = coords[2][i]
            foot_coords_rr = coords[3][i]

            # 보정된 좌표 계산
            calibrated_coords = self.inverse_kinematics.calculate_foot_position_with_orientation(
                *diff_orientation,
                [foot_coords_fl, foot_coords_fr, foot_coords_rl, foot_coords_rr]
            )

            # 보정된 좌표를 리스트에 추가
            calibrated_coords_list[0].append(calibrated_coords["fl_foot"])
            calibrated_coords_list[1].append(calibrated_coords["fr_foot"])
            calibrated_coords_list[2].append(calibrated_coords["rl_foot"])
            calibrated_coords_list[3].append(calibrated_coords["rr_foot"])

        delay = speed/resolution

        for i in range(int(resolution)):

            foot_coords = [calibrated_coords_list[0][i],calibrated_coords_list[1][i],calibrated_coords_list[2][i],calibrated_coords_list[3][i]]
            fl_foot = (np.array(foot_coords[0]) - np.array(hip_pose["fl_hip"]))
            fr_foot = (np.array(foot_coords[1]) - np.array(hip_pose["fr_hip"]))
            rl_foot = (np.array(foot_coords[2]) - np.array(hip_pose["rl_hip"]))
            rr_foot = (np.array(foot_coords[3]) - np.array(hip_pose["rr_hip"]))

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
            # self.motor_controller.move_motors(angle_commands)
            time.sleep(delay)

            feet_positions = {
                "fl_foot": foot_coords[0],
                "fr_foot": foot_coords[1],
                "rl_foot": foot_coords[2],
                "rr_foot": foot_coords[3],
            }
            for foot, position in feet_positions.items():
                self.pose_cmd.update_pose(foot, position)
            print(pose_cmd.get_pose())

if __name__ == "__main__":
    controller = MotionController()

    controller.pose_control(config.config.start_pose, [0, 0, 0], 20)

    controller.move_control(40,40,400,"forward", [0,0,0])
    time.sleep(2)
    # print("보행 완료")

    controller.pose_control(config.config.start_pose, [0, 0, 0], 100)


