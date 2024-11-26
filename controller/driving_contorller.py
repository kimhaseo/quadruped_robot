from solver.trajectory import GaitPatternGenerator
from solver.inverse import Kinematics
from config.config import leg_resolution
from config.motor_cmd import AngleCommand
from controller.motor_controller import MotorController
import time
import math


class RobotController:
    def __init__(self, robot_speed, distance, step_height, motion, orientation):
        self.robot_speed = robot_speed  # mm/s
        self.distance = distance  # mm
        self.step_height = step_height  # mm
        self.motion = motion
        self.orientation = orientation

        self.inverse_kinematics = Kinematics()
        self.motor_controller = MotorController()
        self.pattern_generator = GaitPatternGenerator()

    def calculate_steps(self):
        step_count = self.distance / self.robot_speed
        return step_count

    def generate_foot_poses(self, speed):
        return self.pattern_generator.generate_crawl_gait_pattern(speed, self.step_height, self.motion)

    def motor_control(self, foot_poses, step_count, base_foot_poses):
        num_steps = leg_resolution
        motor_delay = 1 / (leg_resolution * 2)

        for _ in range(step_count):
            for i in range(num_steps):
                start_time = time.perf_counter()

                joint_angles = [
                    self.inverse_kinematics.calculate_joint_angle(False, foot_poses[0][0][i] + base_foot_poses[0][0],
                                                                  foot_poses[0][1][i] + base_foot_poses[0][1],
                                                                  foot_poses[0][2][i] + base_foot_poses[0][2]),
                    self.inverse_kinematics.calculate_joint_angle(True, foot_poses[1][0][i] + base_foot_poses[1][0],
                                                                  foot_poses[1][1][i] + base_foot_poses[1][1],
                                                                  foot_poses[1][2][i] + base_foot_poses[1][2]),
                    self.inverse_kinematics.calculate_joint_angle(False, foot_poses[2][0][i] + base_foot_poses[2][0],
                                                                  foot_poses[2][1][i] + base_foot_poses[2][1],
                                                                  foot_poses[2][2][i] + base_foot_poses[2][2]),
                    self.inverse_kinematics.calculate_joint_angle(True, foot_poses[3][0][i] + base_foot_poses[3][0],
                                                                  foot_poses[3][1][i] + base_foot_poses[3][1],
                                                                  foot_poses[3][2][i] + base_foot_poses[3][2]),
                ]

                angle_commands = [
                    AngleCommand("fl_joint1", -joint_angles[0][0]),
                    AngleCommand("fl_joint2", -joint_angles[0][1]),
                    AngleCommand("fl_joint3", -2 * joint_angles[0][2]),
                    AngleCommand("fr_joint1", joint_angles[1][0]),
                    AngleCommand("fr_joint2", joint_angles[1][1]),
                    AngleCommand("fr_joint3", 2 * joint_angles[1][2]),
                    # Uncomment below lines for rear legs if needed
                    # AngleCommand("rl_joint1", -joint_angles[2][0]),
                    # AngleCommand("rl_joint2", -joint_angles[2][1]),
                    # AngleCommand("rl_joint3", -2 * joint_angles[2][2]),
                    # AngleCommand("rr_joint1", joint_angles[3][0]),
                    # AngleCommand("rr_joint2", joint_angles[3][1]),
                    # AngleCommand("rr_joint3", 2 * joint_angles[3][2]),
                ]

                self.motor_controller.move_motors(angle_commands)

                while time.perf_counter() - start_time < motor_delay:
                    pass

        # Reset to base foot poses
        self.reset_to_base_pose(base_foot_poses)

    def reset_to_base_pose(self, base_foot_poses):
        joint_angles = [
            self.inverse_kinematics.calculate_joint_angle(False, *base_foot_poses[0]),
            self.inverse_kinematics.calculate_joint_angle(True, *base_foot_poses[3]),
        ]

        angle_commands = [
            AngleCommand("fl_joint1", -joint_angles[0][0]),
            AngleCommand("fl_joint2", -joint_angles[0][1]),
            AngleCommand("fl_joint3", -2 * joint_angles[0][2]),
            AngleCommand("rr_joint1", joint_angles[1][0]),
            AngleCommand("rr_joint2", joint_angles[1][1]),
            AngleCommand("rr_joint3", 2 * joint_angles[1][2]),
        ]

        self.motor_controller.move_motors(angle_commands)
        self.motor_controller.close()

    def drive(self):
        base_foot_poses = self.inverse_kinematics.calculate_foot_position_with_orientation(*self.orientation)
        step_count = self.calculate_steps()

        if math.isclose(step_count, round(step_count)):
            step_count = round(step_count)
            foot_poses = self.generate_foot_poses(self.robot_speed)
            self.motor_control(foot_poses, step_count, base_foot_poses)
        elif step_count == 0:
            foot_poses = self.generate_foot_poses(self.robot_speed)
            self.motor_control(foot_poses, 1000, base_foot_poses)
        else:
            step_count_int = math.floor(step_count)
            foot_poses = self.generate_foot_poses(self.robot_speed)
            self.motor_control(foot_poses, step_count_int, base_foot_poses)

            remaining_distance = self.distance - (step_count_int * self.robot_speed)
            adjusted_speed = remaining_distance
            foot_poses = self.generate_foot_poses(adjusted_speed)
            self.motor_control(foot_poses, 1, base_foot_poses)


if __name__ == "__main__":
    controller = RobotController(
        robot_speed=120,
        distance=2400,
        step_height=50,
        motion="forward",
        orientation=[0, 0, 0],
    )
    time.sleep(2)
    controller.drive()
