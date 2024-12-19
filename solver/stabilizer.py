import sys
import os
from ctypes.wintypes import tagRECT

import numpy as np
import threading
import time
from manager.pose_manager import pose_cmd
from sensor.imu import MW_AHRS
from fontTools.misc.arrayTools import offsetRect
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from solver.inverse import Kinematics

target_orientation = [0,0,0]

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
            print(diff_orientation)
            time.sleep(0.2)




if __name__ == "__main__":

    stabilizer = StabilizerSolver()
    stabilizer.stabilize(target_orientation)
