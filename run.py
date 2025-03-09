import threading
import time
from controller.joystick import Joystick  # joystick.py에서 Joystick 클래스
from controller.main_cotroller import MainLoop  # main_control.py에서 제어 루프 함수


def run_joystick():
    joy = Joystick()
    joy.main()  # 키보드 입력 감지 루프 실행

def run_control():
    # main_control.py에 정의한 제어 루프 함수 실행
    main = MainLoop()
    main.run()


if __name__ == "__main__":
    t1 = threading.Thread(target=run_joystick)
    t2 = threading.Thread(target=run_control)

    t1.start()
    t2.start()

    t1.join()
    t2.join()