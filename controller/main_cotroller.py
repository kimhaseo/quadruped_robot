import time
from manager.joystick_manger import control_cmd
from manager.pose_manager import pose_cmd

class MainLoop:
    def __init__(self):
        pass

    def run(self):
        while True:
            current_pose = pose_cmd.get_pose()
            current_vel = current_pose['body_vel']
            current_orientation = current_pose['body_orientation']
            print(current_vel,current_orientation)
            print(control_cmd.get_control_cmd())
            time.sleep(0.1)

if __name__ == "__main__":
    main_loop = MainLoop()
    main_loop.run()