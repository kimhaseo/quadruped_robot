from can_handler import CanHandler

class MotorController:
    def __init__(self):
        self.can_handler = CanHandler()

    def move_motor_to_angle(self, motor_command):
        try:
            angle = motor_command.angle
            can_id = motor_command.can_id
            angle_control = int(angle * 1000)
            command_byte = 0xA3
            null_byte = 0x00
            angle_control_low = angle_control & 0xFF
            angle_control_mid1 = (angle_control >> 8) & 0xFF
            angle_control_mid2 = (angle_control >> 16) & 0xFF
            angle_control_high = (angle_control >> 24) & 0xFF

            data = [
                command_byte,
                null_byte,
                null_byte,
                null_byte,
                angle_control_low,
                angle_control_mid1,
                angle_control_mid2,
                angle_control_high
            ]

            # CAN 메시지 전송
            self.can_handler.send_message(can_id, data)
            print(f"Motor {motor_command.motor_name} moved to angle {angle} degrees.")

        except Exception as e:
            print(f"Error moving motor {motor_command.motor_name}: {e}")
            raise

    def move_motors(self, motor_commands):
        for motor_command in motor_commands:
            self.move_motor_to_angle(motor_command)

    def close(self):
        """CAN 인터페이스를 종료하고 버스를 안전하게 닫습니다."""
        self.can_handler.close()  # CAN 핸들러 종료