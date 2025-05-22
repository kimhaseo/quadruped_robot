from dataclasses import dataclass

# 공통된 부분을 담당하는 기본 클래스
class MotorCommand:
    motor_mapping = {
        "fl_joint1": 0x141,
        "fl_joint2": 0x142,
        "fl_joint3": 0x143,
        "fr_joint1": 0x144,
        "fr_joint2": 0x145,
        "fr_joint3": 0x146,
        "rl_joint1": 0x147,
        "rl_joint2": 0x148,
        "rl_joint3": 0x149,
        "rr_joint1": 0x14A,
        "rr_joint2": 0x14B,
        "rr_joint3": 0x14C
    }

    def __init__(self, motor_name: str, value: int):
        self.motor_name = motor_name
        self.value = value
        self.can_id = self.get_can_id(self.motor_name)

    def get_can_id(self, motor_name: str):
        return self.motor_mapping.get(motor_name, None)


# AngleCommand 클래스는 MotorCommand를 상속받아서 간단히 생성
@dataclass
class AngleCommand(MotorCommand):
    angle: int
    speed: int

    def __init__(self, motor_name: str, angle: int, speed: int = 400):
        # MotorCommand는 motor_name과 value를 받으므로 angle을 value로 사용
        super().__init__(motor_name, angle)
        self.angle = angle  # angle 속성 설정
        self.speed = speed  # speed 속성 설정


# AeccelCommand 클래스도 MotorCommand를 상속받아서 작성
@dataclass
class AeccelCommand(MotorCommand):
    Aeccel: int

    def __init__(self, motor_name: str, Aeccel: int):
        # MotorCommand는 motor_name과 value를 받으므로 Aeccel을 value로 사용
        super().__init__(motor_name, Aeccel)


@dataclass
class PidCommand(MotorCommand):
    p_gain: int
    i_gain: int

    def __init__(self, motor_name: str, p_gain: int, i_gain: int):
        # MotorCommand는 motor_name과 value를 받으므로 p_gain을 value로 사용
        super().__init__(motor_name, p_gain)
        self.p_gain = p_gain
        self.i_gain = i_gain