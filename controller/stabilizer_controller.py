import sys
import os
import numpy as np

from fontTools.misc.arrayTools import offsetRect

# Import paths
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from solver.inverse import Kinematics

imu_data = [0,0,[0,0,0]]

# from sensor.imu import AHRSProcessor


class StabilizerController:
    def __init__(self):

        self.kinematics = Kinematics()
        self.imu_data = imu_data

    def stabilize(self, target_orientation):

        imu_acceleration = self.imu_data[0]
        imu_angular_vel = self.imu_data[1]
        imu_orientation = self.imu_data[2]

        # Calculate orientation gain
        orientation_gain = list(np.array(target_orientation) - np.array(imu_orientation))
        calibrated_orientation = list(np.array(target_orientation) + np.array(orientation_gain))

        # Calculate the target pose using the kinematics module
        target_pose = self.kinematics.calculate_foot_position_with_orientation(*calibrated_orientation)

        return target_pose


if __name__ == "__main__":
    # Initialize kinematics and IMU data
    kinematics = Kinematics()
    imu_data = [0, 0, [0, 0, 0]]  # Example IMU data

    # Create StabilizerControl instance
    stabilizer = StabilizerController(kinematics, imu_data)

    # Define the target orientation
    target_orientation = [0, -10, 0]

    # Perform stabilization
    target_pose = stabilizer.stabilize(target_orientation)

    print("Target Pose:", target_pose)