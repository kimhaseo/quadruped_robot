import sys
import os
import time
from solver.stabilizer import StabilizerSolver
from controller.motion_controller import MotionController
from config.config import init_pose

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

stabilizer_controller = StabilizerSolver()
motion_controller = MotionController()
target_orientation = [0,10,0]

def main():

    # target_foot_coord = stabilizer_controller.stabilize(target_orientation)
    motion_controller.pose_control(init_pose)
    time.sleep(5)
    target_foot_coord = stabilizer_controller.stabilize(target_orientation)
    motion_controller.pose_control(target_foot_coord)
    time.sleep(5)
    motion_controller.move_contorl(30,20,50,"forward")
    time.sleep(5)

    # while True:
    #
    #     target_foot_coord = stabilizer_controller.stabilize(target_orientation)
    #     motion_controller.pose_control(target_foot_coord)
        # print(target_foot_coord)

if __name__ == "__main__":

    main()