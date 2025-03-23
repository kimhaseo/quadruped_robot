import threading
import time
from controller.joystick import Joystick  # joystick.py에서 Joystick 클래스
from controller.main_cotroller import MainLoop  # main_control.py에서 제어 루프 함수
from solver.velocity_calc import VelocityEstimator  # 아까 만든 속도 추정 클래스

def run_joystick():
    """ 조이스틱 입력 감지 실행 """
    joy = Joystick()
    joy.main()

def run_control():
    """ 로봇 제어 루프 실행 """
    main = MainLoop()
    main.run()

def run_velocity_estimator():
    """ 속도 추정기 실행 """
    estimator = VelocityEstimator(dt=0.01)
    estimator.start()

    try:
        while True:
            v_com, omega = estimator.get_velocity()
            print("로봇의 선속도 (x, y, z):", v_com)
            print("로봇의 각속도 (rad/s, x, y, z):", omega)
            time.sleep(0.1)  # 0.1초마다 속도 출력
    except KeyboardInterrupt:
        estimator.stop()

if __name__ == "__main__":
    try:
        t1 = threading.Thread(target=run_joystick, daemon=True)
        t2 = threading.Thread(target=run_control, daemon=True)
        t3 = threading.Thread(target=run_velocity_estimator, daemon=True)

        t1.start()
        t2.start()
        t3.start()

        while True:
            time.sleep(1)  # 메인 스레드가 종료되지 않도록 유지

    except KeyboardInterrupt:
        print("\n[종료] 프로그램이 안전하게 종료됩니다.")
