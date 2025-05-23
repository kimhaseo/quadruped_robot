import time
from ctypes.wintypes import tagRECT

from manager.joystick_manger import control_cmd
from manager.pose_manager import pose_cmd
from controller.motion_controller import MotionController
import config.config
class MainLoop:
    def __init__(self):
        self.motion_controller = MotionController()
        pass

    def run(self):
        target_pose = config.config.init_pose
        self.motion_controller.pose_control(target_pose, [0, 0, 0], 1)
        while True:
            current_pose = pose_cmd.get_pose()
            control_pose = control_cmd.get_control_cmd()
            current_vel = current_pose['body_vel']
            current_orientation = current_pose['body_orientation']
            time.sleep(0.01)
            if current_vel[0] != control_pose["linear_velocity"]:
                target_speed = control_pose["linear_velocity"] - current_vel[0]
                if target_speed > 0:
                    motion = "forward"
                elif target_speed < 0:
                    motion = "backward"

                self.motion_controller.move_control(target_speed, 70, motion, [0, 0, 0])


if __name__ == "__main__":
    main_loop = MainLoop()
    main_loop.run()