import config.config
from solver.trajectory import TrajectoryGenerator
from solver.inverse import Kinematics
from config.config import leg_resolution,hip_pose
from config.motor_cmd import AngleCommand
from controller.motor_controller import MotorController
from config.pose_cmd import PoseCommand
import time
import math
import numpy as np


class MotionController:
    def __init__(self):
        self.inverse_kinematics = Kinematics()
        # self.motor_controller = MotorController()
        self.trajectory_generator = TrajectoryGenerator()
        self.pose_command = PoseCommand()

    # def calculate_steps(self, robot_speed, distance):
    #     step_count = distance / robot_speed
    #     return step_count
    #
    # def generate_foot_poses(self, speed, step_height, motion):
    #     return self.trajectory_generator.generate_move_trajectory(speed, step_height, motion)
    #
    # def motor_control(self, foot_poses, step_count, base_foot_poses):
    #     num_steps = leg_resolution
    #     motor_delay = 1 / (leg_resolution * 1.5)
    #
    #     for _ in range(step_count):
    #         for i in range(num_steps):
    #             start_time = time.perf_counter()
    #
    #             joint_angles = [
    #                 self.inverse_kinematics.calculate_joint_angle(False, foot_poses[0][0][i] + base_foot_poses[0][0],
    #                                                               foot_poses[0][1][i] + base_foot_poses[0][1],
    #                                                               foot_poses[0][2][i] + base_foot_poses[0][2]),
    #                 self.inverse_kinematics.calculate_joint_angle(True, foot_poses[1][0][i] + base_foot_poses[1][0],
    #                                                               foot_poses[1][1][i] + base_foot_poses[1][1],
    #                                                               foot_poses[1][2][i] + base_foot_poses[1][2]),
    #                 self.inverse_kinematics.calculate_joint_angle(False, foot_poses[2][0][i] + base_foot_poses[2][0],
    #                                                               foot_poses[2][1][i] + base_foot_poses[2][1],
    #                                                               foot_poses[2][2][i] + base_foot_poses[2][2]),
    #                 self.inverse_kinematics.calculate_joint_angle(True, foot_poses[3][0][i] + base_foot_poses[3][0],
    #                                                               foot_poses[3][1][i] + base_foot_poses[3][1],
    #                                                               foot_poses[3][2][i] + base_foot_poses[3][2]),
    #             ]
    #
    #             angle_commands = [
    #                 AngleCommand("fl_joint1", -joint_angles[0][0]),
    #                 AngleCommand("fl_joint2", -joint_angles[0][1]),
    #                 AngleCommand("fl_joint3", -2 * joint_angles[0][2]),
    #                 AngleCommand("fr_joint1", joint_angles[1][0]),
    #                 AngleCommand("fr_joint2", joint_angles[1][1]),
    #                 AngleCommand("fr_joint3", 2 * joint_angles[1][2]),
    #             ]
    #
    #             self.motor_controller.move_motors(angle_commands)
    #
    #             while time.perf_counter() - start_time < motor_delay:
    #                 pass
    #
    # def joint_control(self,target_foot_poses):
    #     num_steps = leg_resolution
    #     motor_delay = 1 / (leg_resolution * 1.5)
    #
    #     for i in range(num_steps):
    #         start_time = time.perf_counter()
    #
    #         joint_angles = [
    #             self.inverse_kinematics.calculate_joint_angle(False,target_foot_poses[0][0],target_foot_poses[0][1],target_foot_poses[0][2]),
    #             self.inverse_kinematics.calculate_joint_angle(False, target_foot_poses[1][0], target_foot_poses[1][1],target_foot_poses[1][2]),
    #             self.inverse_kinematics.calculate_joint_angle(False, target_foot_poses[2][0], target_foot_poses[2][1],target_foot_poses[2][2]),
    #             self.inverse_kinematics.calculate_joint_angle(False, target_foot_poses[3][0], target_foot_poses[3][1],target_foot_poses[3][2])
    #             ]
    #
    #
    #
    #         angle_commands = [
    #             AngleCommand("fl_joint1", -joint_angles[0][0]),
    #             AngleCommand("fl_joint2", -joint_angles[0][1]),
    #             AngleCommand("fl_joint3", -2 * joint_angles[0][2]),
    #             AngleCommand("fr_joint1", joint_angles[1][0]),
    #             AngleCommand("fr_joint2", joint_angles[1][1]),
    #             AngleCommand("fr_joint3", 2 * joint_angles[1][2]),
    #         ]
    #
    #         self.motor_controller.move_motors(angle_commands)
    #
    #         while time.perf_counter() - start_time < motor_delay:
    #             pass
    #
    # def drive(self, robot_speed, distance, step_height, motion, orientation):
    #     base_foot_poses = self.inverse_kinematics.calculate_foot_position_with_orientation(*orientation)
    #     step_count = self.calculate_steps(robot_speed, distance)
    #
    #     if math.isclose(step_count, round(step_count)):
    #         step_count = round(step_count)
    #         foot_poses = self.generate_foot_poses(robot_speed, step_height, motion)
    #         self.motor_control(foot_poses, step_count, base_foot_poses)
    #     elif step_count == 0:
    #         foot_poses = self.generate_foot_poses(robot_speed, step_height, motion)
    #         self.motor_control(foot_poses, 1000, base_foot_poses)
    #     else:
    #         step_count_int = math.floor(step_count)
    #         foot_poses = self.generate_foot_poses(robot_speed, step_height, motion)
    #         self.motor_control(foot_poses, step_count_int, base_foot_poses)
    #
    #         remaining_distance = distance - (step_count_int * robot_speed)
    #         adjusted_speed = remaining_distance
    #         foot_poses = self.generate_foot_poses(adjusted_speed, step_height, motion)
    #         self.motor_control(foot_poses, 1, base_foot_poses)


    "---------------------------"

    def pose_control(self, target_pose,speed):

        current_pose = self.pose_command.get_pose()
        coords = self.trajectory_generator.generate_pose_trajectory(target_pose,current_pose)
        # print(current_pose)
        self.joint_control(coords,speed,leg_resolution)

    def move_contorl(self, speed, step_hight, distance, robot_motion):
        print(distance)
        coords = self.trajectory_generator.generate_move_trajectory(speed, step_hight, robot_motion)
        self.joint_control(coords, speed, leg_resolution)

    def joint_control(self,coords,speed,resolution):

        delay = 0.7 / (resolution/speed)

        for i in range(resolution):

            fl_foot = (np.array(coords[0][i]) - np.array(hip_pose["fl_hip"]))
            fr_foot = (np.array(coords[1][i]) - np.array(hip_pose["fr_hip"]))
            rl_foot = (np.array(coords[2][i]) - np.array(hip_pose["rl_hip"]))
            rr_foot = (np.array(coords[3][i]) - np.array(hip_pose["rr_hip"]))


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
            "fl_foot": fl_foot,
            "fr_foot": fr_foot,
            "rl_foot": rl_foot,
            "rr_foot": rr_foot,
        }

        for foot, position in feet_positions.items():
            self.pose_command.update_pose(foot, position)

        print(self.pose_command.get_pose)

if __name__ == "__main__":
    controller = MotionController()
    target_pose = config.config.init_pose
    # controller.pose_control(target_pose,1)
    controller.move_contorl(10,20,10,"forward")

