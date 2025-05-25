import time
from ctypes.wintypes import tagRECT
import threading
from manager.joystick_manger import control_cmd
from manager.pose_manager import pose_cmd
from controller.motion_controller import MotionController
import config.config
from controller.joystick import Joystick
class MainLoop:
    def __init__(self):
        self.motion_controller = MotionController()
        pass

    def run(self):

        self.motion_controller.pose_control(config.config.start_pose, [0, 0, 0], 20)
        self.motion_controller.pose_control(config.config.init_pose, [0, 0, 0], 100)

        while True:

            current_pose = pose_cmd.get_pose()
            control_pose = control_cmd.get_control_cmd()
            current_vel = current_pose['body_vel']
            time.sleep(0.01)

            if current_vel[0] != control_pose["linear_velocity"]:
                target_speed = control_pose["linear_velocity"] - current_vel[0]
                target_speed = int(target_speed)

                if target_speed > 0:
                    motion = "forward"
                elif target_speed < 0:
                    motion = "backward"

                self.motion_controller.move_control(target_speed, 70, target_speed , motion, [0, 0, 0])


if __name__ == "__main__":
    main_loop = MainLoop()
    joystick = Joystick()

    t1 = threading.Thread(target=main_loop.run)
    t2 = threading.Thread(target=joystick.main)

    t1.start()
    t2.start()

    # 메인 스레드가 두 스레드 종료까지 기다림 (필요 시)
    t1.join()
    t2.join()