import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from controller.can_handler import CanHandler

class MotorController:
    def __init__(self):
        self.can_handler = CanHandler()

    def move_motor_to_angle(self, angle_command):
        try:
            angle = angle_command.angle
            speed = angle_command.speed
            can_id = angle_command.can_id
            angle_control = int(angle * 1000)
            command_byte = 0xA4
            null_byte = 0x00
            speed_limit_low = speed & 0xFF
            speed_limit_high = (speed >> 8) & 0xFF
            angle_control_low = angle_control & 0xFF
            angle_control_mid1 = (angle_control >> 8) & 0xFF
            angle_control_mid2 = (angle_control >> 16) & 0xFF
            angle_control_high = (angle_control >> 24) & 0xFF



            data = [
                command_byte,
                null_byte,
                speed_limit_low,
                speed_limit_high,
                angle_control_low,
                angle_control_mid1,
                angle_control_mid2,
                angle_control_high
            ]

            # CAN 메시지 전송
            self.can_handler.send_message(can_id, data)
            print(f"Motor {angle_command.motor_name} moved to angle {angle} degrees.")

        except Exception as e:
            print(f"Error moving motor {angle_command.motor_name}: {e}")
            raise

    def move_motors(self, angle_commands):
        for angle_command in angle_commands:
            self.move_motor_to_angle(angle_command)

    def close(self):
        """CAN 인터페이스를 종료하고 버스를 안전하게 닫습니다."""
        self.can_handler.close()  # CAN 핸들러 종료

    def read_acceleration(self,id):

        can_id = id
        command_byte = 0x33
        null_byte = 0x00

        data = [
            command_byte,
            null_byte,
            null_byte,
            null_byte,
            null_byte,
            null_byte,
            null_byte,
            null_byte
        ]

        # CAN 메시지 전송
        self.can_handler.send_message(can_id, data)
        response = self.can_handler.receive_message()

        return response

    def write_pid_gain(self, pid_command):
        try:
            p_gain = pid_command.p_gain
            i_gain = pid_command.i_gain
            can_id = pid_command.can_id

            # 8비트 값 그대로 사용
            data = [
                0x31,  # command byte
                0x00,  # NULL byte
                int(p_gain),  # Position loop P parameter (8-bit)
                int(i_gain),  # Position loop I parameter (8-bit)
                0x00,  # Speed loop P parameter (0x00은 예시)
                0x00,  # Speed loop I parameter
                0x00,  # Torque loop P parameter
                0x00  # Torque loop I parameter
            ]

            # CAN 메시지 전송
            self.can_handler.send_message(can_id, data)
            print(f"PID Gains sent: P_gain={p_gain}, I_gain={i_gain}")
        except Exception as e:
            print(f"Error sending PID gains: {e}")

    def write_acceleration(self,accel_command):
        try:

            accel = accel_command.angle
            can_id = accel_command.can_id
            accel_control = int(accel)
            if accel_control > 100:
                accel_control = 100
            command_byte = 0x34
            null_byte = 0x00
            accel_control_low = accel_control & 0xFF
            accel_control_mid1 = (accel_control >> 8) & 0xFF
            accel_control_mid2 = (accel_control >> 16) & 0xFF
            accel_control_high = (accel_control >> 24) & 0xFF

            data = [
                command_byte,
                null_byte,
                null_byte,
                null_byte,
                accel_control_low,
                accel_control_mid1,
                accel_control_mid2,
                accel_control_high
            ]

            # CAN 메시지 전송
            self.can_handler.send_message(can_id, data)
            print(f"Motor {accel_command.motor_name} translation  {accel} degree/sec.")

        except Exception as e:
            print(f"Error moving motor {accel_command.motor_name}: {e}")
            raise