import sys
import os
import time
from config.pose_cmd import PoseCommand
from controller.stabilizer_controller import StabilizerController
from controller.motion_controller import MotionController
from config.config import init_pose

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

stabillizer_controller = StabilizerController()
motion_controller = MotionController()
taget_orientation = [0,0,0]

def main():

    # target_foot_coord = stabillizer_controller.stabilize(taget_orientation)
    motion_controller.pose_control(init_pose)

    motion_controller.move_contorl(30,20,50,"forward")

    # while True:
    #
    #     target_foot_coord = stabillizer_controller.stabilize(taget_orientation)
    #     motion_controller.pose_control(target_foot_coord)
        # print(target_foot_coord)

if __name__ == "__main__":

    main()