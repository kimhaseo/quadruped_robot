import sys
import os
from ctypes.wintypes import tagRECT

import numpy as np
import threading
import time
from manager.pose_manager import pose_cmd
from sensor.ahrs import MW_AHRS
from fontTools.misc.arrayTools import offsetRect
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from solver.inverse import Kinematics

target_orientation = [0,10,0]

class StabilizerSolver:
    def __init__(self):

        self.kinematics = Kinematics()
        self.ahrs = MW_AHRS()
        self.imu_thread = threading.Thread(target=self.ahrs.pose_update, daemon=True)
        self.imu_thread.start()

    def stabilize(self,target_orientation):

        while True:
            body_pose = pose_cmd.get_pose()
            current_orientation = body_pose["body_orientation"]
            diff_orientation = np.array(target_orientation) - (np.array(current_orientation))

            if any(30 > value > 1 for value in diff_orientation):
                pass

            elif any(value >= 20 for value in diff_orientation):
                raise ValueError ("자세 이상")

            else:
                diff_orientation = [0,0,0]

            calibration_orientation = np.array(target_orientation) + (np.array(diff_orientation))
            coords = self.kinematics.calculate_foot_position_with_orientation(*calibration_orientation)

            feet_positions = {
                "fl_foot": coords["fl_foot"],
                "fr_foot": coords["fr_foot"],
                "rl_foot": coords["rl_foot"],
                "rr_foot": coords["rr_foot"],
            }
            for foot, position in feet_positions.items():
                pose_cmd.update_pose(foot, position)
            time.sleep(0.1)

            print(pose_cmd.get_pose())

if __name__ == "__main__":

    stabilizer = StabilizerSolver()
    stabilizer.stabilize(target_orientation)
