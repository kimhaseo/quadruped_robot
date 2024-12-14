import sys
import os
import time

from config.pose_cmd import PoseCommand

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from controller.motion_controller import MotionController
from solver.trajectory import TrajectoryGenerator
from config.motor_cmd import MotorCommand, AngleCommand
from manager.pose_manager import pose_cmd
from controller.stabilizer_controller import StabilizerController
from controller.motion_controller import MotionController

stabillizer_controller = StabilizerController()
motion_controller = MotionController()
taget_orientation = [0,10,0]

def main():
    print(pose_cmd.get_pose())
   
    target_foot_coord= stabillizer_controller.stabilize(taget_orientation)

    motion_controller.pose_control(target_foot_coord)
    # print("자세변경 완료")
    # time.sleep(2)
    # motion_controller.move_contorl(30,20,50,"forward")
    # print("마지막 포즈")

    print(pose_cmd.get_pose())

if __name__ == "__main__":

    main()