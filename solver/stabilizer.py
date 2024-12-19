import sys
import os
from ctypes.wintypes import tagRECT

import numpy as np
import threading
import time

from numpy.f2py.crackfortran import kindselector

from manager.pose_manager import pose_cmd
from sensor.ahrs import MW_AHRS
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

            if any(15 > abs(value) > 1 for value in diff_orientation):

                calibartion_orientation = np.array(target_orientation) + np.array(diff_orientation)
                coords = self.kinematics.calculate_foot_position_with_orientation(*calibartion_orientation)
                print(coords)

            elif any(abs(value) >= 15 for value in diff_orientation):
                raise ValueError ("자세 이상")

            else:
                pass

            print(diff_orientation)

if __name__ == "__main__":

    stabilizer = StabilizerSolver()
    stabilizer.stabilize(target_orientation)
