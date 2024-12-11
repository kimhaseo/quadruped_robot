import threading
import math
import serial
import time

# AHRS 센서 데이터를 처리하는 클래스
class AHRSProcessor:
    def __init__(self, port, baudrate):
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.run = True
        self.acc = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.gyo = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.orientation = {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}

    def parse_data(self, data):
        msg_type = data[1]
        if msg_type == 0x01:  # ACC
            acc_x = int.from_bytes(data[2:4], 'little', signed=True)
            acc_y = int.from_bytes(data[4:6], 'little', signed=True)
            acc_z = int.from_bytes(data[6:8], 'little', signed=True)

            self.acc['x'] = acc_x / 1000.0 * 9.8
            self.acc['y'] = acc_y / 1000.0 * 9.8
            self.acc['z'] = acc_z / 1000.0 * 9.8

        elif msg_type == 0x02:  # GYO
            gyo_x = int.from_bytes(data[2:4], 'little', signed=True)
            gyo_y = int.from_bytes(data[4:6], 'little', signed=True)
            gyo_z = int.from_bytes(data[6:8], 'little', signed=True)

            self.gyo['x'] = gyo_x / 10.0 * 0.01745
            self.gyo['y'] = gyo_y / 10.0 * 0.01745
            self.gyo['z'] = gyo_z / 10.0 * 0.01745

        elif msg_type == 0x03:  # DEG
            deg_x = int.from_bytes(data[2:4], 'little', signed=True)
            deg_y = int.from_bytes(data[4:6], 'little', signed=True)
            deg_z = int.from_bytes(data[6:8], 'little', signed=True)

            x = deg_x / 100.0
            y = deg_y / 100.0
            z = deg_z / 100.0

            cos_x = math.cos(x)
            sin_x = math.sin(x)
            cos_y = math.cos(y)
            sin_y = math.sin(y)
            cos_z = math.cos(z)
            sin_z = math.sin(z)

            self.orientation['w'] = (cos_z * cos_y * cos_x) + (sin_z * sin_y * sin_x)
            self.orientation['x'] = (cos_z * cos_y * sin_x) - (sin_z * sin_y * cos_x)
            self.orientation['y'] = (cos_z * sin_y * cos_x) + (sin_z * cos_y * sin_x)
            self.orientation['z'] = (sin_z * cos_y * cos_x) - (cos_z * sin_y * sin_x)

    def read_data(self):
        while self.run:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(10)  # 데이터 길이를 적절히 설정
                if len(data) == 10:
                    self.parse_data(data)

    def stop(self):
        self.run = False
        self.serial_port.close()

# 메인 코드
if __name__ == "__main__":
    processor = AHRSProcessor('/dev/ttyUSB0', 115200)
    thread = threading.Thread(target=processor.read_data)

    try:
        thread.start()
        while True:
            print("Acceleration:", processor.acc)
            print("Angular Velocity:", processor.gyo)
            print("Orientation:", processor.orientation)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping...")
        processor.stop()
        thread.join()