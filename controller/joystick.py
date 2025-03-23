from pynput import keyboard
from manager.joystick_manger import control_cmd

class Joystick:
    def __init__(self):
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)

    def on_press(self, key):
        """키가 눌렸을 때"""
        try:
            linear_delta = 0.0
            angular_delta = 0.0
            orientation_delta = [0.0, 0.0, 0.0]

            if key.char == "w":
                linear_delta = 50
            elif key.char == "s":
                linear_delta = -50
            elif key.char == "a":
                angular_delta = 20
            elif key.char == "d":
                angular_delta = -20
            elif key.char == "q":
                orientation_delta[0] = 3.14/360
            elif key.char == "e":
                orientation_delta[0] = -3.14/360
            elif key.char == "k":
                linear_delta = 0
                angular_delta = 0

            #  함수 호출
            control_cmd.update_(linear_delta=linear_delta, angular_delta=angular_delta, orientation_delta=orientation_delta)
            print(control_cmd.get_control_cmd())

        except AttributeError:
            pass  # 특수키 (Shift, Ctrl 등) 무시

    def on_release(self, key):
        """키가 떼졌을 때"""
        if key == keyboard.Key.esc:  # ESC 키로 종료
            return False

    def main(self):
        with self.listener:
            self.listener.join()  # 키 입력 감지 루프

if __name__ == "__main__":
    joy = Joystick()
    joy.main()
