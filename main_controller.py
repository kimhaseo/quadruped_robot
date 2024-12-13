import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from controller.motion_controller import MotionController
from solver.trajectory import TrajectoryGenerator
from config.motor_cmd import MotorCommand, AngleCommand
from controller.stabilizer_controller import StabilizerController
from controller.motion_controller import MotionController

stabillizer_controller = StabilizerController()
motion_controller = MotionController()

taget_orientation = [0,0,0]

def main():

    target_foot_coord= stabillizer_controller.stabilize(taget_orientation)
    print(target_foot_coord['fl_foot'],target_foot_coord['fr_foot'],target_foot_coord['rl_foot'],target_foot_coord['rr_foot'])
    motion_controller.pose_control(target_foot_coord)


if __name__ == "__main__":

    main()