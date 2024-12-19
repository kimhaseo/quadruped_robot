import serial
import time
import re

from fontTools.misc.arrayTools import offsetRect


class MW_AHRS:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connection = None

    def connect(self):
        """Establishes a connection to the serial port."""
        try:
            self.connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            print(f"Connected to {self.port}")
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            self.connection = None

    def send_command(self, command):
        """Sends a command to the sensor."""
        if self.connection:
            try:
                # Append CR (0x0D) and LF (0x0A) to the command
                full_command = command + '\r\n'
                self.connection.write(full_command.encode())
                print(f"Sent: {command}")
            except Exception as e:
                print(f"Error sending command: {e}")

    def read_response(self):
        """Reads the response from the sensor."""
        if self.connection:
            try:
                response = self.connection.readline().decode().strip()
                # print(f"Received: {response}")
                return response
            except Exception as e:
                print(f"Error reading response: {e}")
                return None

    def disconnect(self):
        """Closes the serial connection."""
        if self.connection:
            self.connection.close()
            print(f"Disconnected from {self.port}")

def parse_angles(response):

    # Use regular expression to extract all floating-point numbers
    angles = re.findall(r"-?\d+\.\d+", response)
    # Convert extracted strings to floats and return as a list
    return [float(angle) for angle in angles]

# Example usage
if __name__ == "__main__":
    # Replace 'COM5' with the appropriate port
    sensor = MW_AHRS(port='/dev/tty.usbserial-B000CVDX')
    sensor.connect()
    sensor.send_command("zro")

    while True:
    # Request Euler angles
        sensor.send_command("ang")
        str_orientation = sensor.read_response()
        orientation = parse_angles(str_orientation)
        print(orientation)
        time.sleep(0.1)  # Allow time for a response

    # Disconnect after use
    sensor.disconnect()