from dataclasses import dataclass


@dataclass
class MotorCommand:
    motor_name: str
    angle: int
    can_id: int = None

    def __post_init__(self):
        # motor_name에 맞는 can_id를 설정
        self.can_id = self.get_can_id(self.motor_name)

    def get_can_id(self, motor_name: str):
        # motor_name에 맞는 can_id 값을 반환하는 메소드
        motor_mapping = {
            "lf_joint1": 0x141,
            # "lf_joint2": 0x142,
            # "lf_joint3": 0x143,
            # "rf_joint1": 0x144,
            "rf_joint2": 0x145,
            # "rf_joint3": 0x146,
            # "lr_joint1": 0x147,
            # "lr_joint2": 0x148,
            # "lr_joint3": 0x149,
            # "rr_joint1": 0x150,
            # "rr_joint2": 0x151,
            # "rr_joint3": 0x152
        }

        return motor_mapping.get(motor_name, None)