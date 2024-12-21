import sys
import os
from ctypes.wintypes import tagRECT
from os import close
from symtable import Symbol

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

    def pose_control(self, target_pose, target_orientation):

        current_pose = self.pose_cmd.get_pose()
        coords = self.trajectory_generator.generate_pose_trajectory(target_pose,current_pose)
        self.joint_control(coords,leg_resolution,2, target_orientation)

    def move_contorl(self, speed, step_hight, distance, robot_motion,target_orientation):
        current_pose = self.pose_cmd.get_pose()
        coords = self.trajectory_generator.generate_move_trajectory(speed, step_hight, robot_motion)
        coords = list(coords)
        step_count, last_step_length = divmod(distance, speed)
        for i in range(step_count):
            coords[0] = (np.array(current_pose['fl_foot']) + coords[0])
            coords[1] = (np.array(current_pose['fr_foot']) + coords[1])
            coords[2] = (np.array(current_pose['rl_foot']) + coords[2])
            coords[3] = (np.array(current_pose['rr_foot']) + coords[3])
            self.joint_control(coords, leg_resolution,1,target_orientation)

        if last_step_length != 0:
            current_pose = self.pose_cmd.get_pose()
            coords = self.trajectory_generator.generate_move_trajectory(last_step_length, step_hight, robot_motion)
            coords = list(coords)
            coords[0] = (np.array(current_pose['fl_foot']) + coords[0])
            coords[1] = (np.array(current_pose['fr_foot']) + coords[1])
            coords[2] = (np.array(current_pose['rl_foot']) + coords[2])
            coords[3] = (np.array(current_pose['rr_foot']) + coords[3])

            self.joint_control(coords, leg_resolution, 1, target_orientation)
        else :
            pass

    def joint_control(self,coords,resolution,speed,target_orientation):

        diff_orientation = -(self.stabilizer.stabilize(target_orientation))
        delay = 1 /resolution/speed
        for i in range(resolution):

            # fl_foot = (np.array(coords[0][i]) - np.array(hip_pose["fl_hip"]))
            # fr_foot = (np.array(coords[1][i]) - np.array(hip_pose["fr_hip"]))
            # rl_foot = (np.array(coords[2][i]) - np.array(hip_pose["rl_hip"]))
            # rr_foot = (np.array(coords[3][i]) - np.array(hip_pose["rr_hip"]))

            foot_coords = [coords[0][i],coords[1][i],coords[2][i],coords[3][i]]
            calebration_foot_coords = self.inverse_kinematics.calculate_foot_position_with_orientation(*diff_orientation,foot_coords)


            fl_foot = (np.array(calebration_foot_coords["fl_foot"]) - np.array(hip_pose["fl_hip"]))
            fr_foot = (np.array(calebration_foot_coords["fr_foot"]) - np.array(hip_pose["fr_hip"]))
            rl_foot = (np.array(calebration_foot_coords["rl_foot"]) - np.array(hip_pose["rl_hip"]))
            rr_foot = (np.array(calebration_foot_coords["rr_foot"]) - np.array(hip_pose["rr_hip"]))

            fl_degree1, fl_degree2, fl_degree3 = self.inverse_kinematics.calculate_joint_angle(False, *fl_foot)
            fr_degree1, fr_degree2, fr_degree3 = self.inverse_kinematics.calculate_joint_angle(True, *fr_foot)
            rl_degree1, rl_degree2, rl_degree3 = self.inverse_kinematics.calculate_joint_angle(False, *rl_foot)
            rr_degree1, rr_degree2, rr_degree3 = self.inverse_kinematics.calculate_joint_angle(True, *rr_foot)

            angle_commands = [
                AngleCommand("fl_joint1", -fl_degree1),
                AngleCommand("fl_joint2", -fl_degree2),
                AngleCommand("fl_joint3", -2 * fl_degree3),
                AngleCommand("fr_joint1", fr_degree1),
                AngleCommand("fr_joint2", fr_degree2),
                AngleCommand("fr_joint3", 2 * fr_degree3),
                AngleCommand("rl_joint1", -rl_degree1),
                AngleCommand("rl_joint2", -rl_degree2),
                AngleCommand("rl_joint3", -2 * rl_degree3),
                AngleCommand("rr_joint1", rr_degree1),
                AngleCommand("rr_joint2", rr_degree2),
                AngleCommand("rr_joint3", 2 * rr_degree3),
            ]
            # self.motor_controller.move_motors(angle_commands)
            time.sleep(delay)

            feet_positions = {
                "fl_foot": calebration_foot_coords["fl_foot"],
                "fr_foot": calebration_foot_coords["fr_foot"],
                "rl_foot": calebration_foot_coords["rl_foot"],
                "rr_foot": calebration_foot_coords["rr_foot"],
            }
            for foot, position in feet_positions.items():
                self.pose_cmd.update_pose(foot, position)
            print(pose_cmd.get_pose())

if __name__ == "__main__":
    controller = MotionController()
    target_pose = config.config.init_pose
    controller.pose_control(target_pose,[0,5,0])
    print("보행 시작")
    controller.move_contorl(50,30,50,"forward", [0,0,0])

