import can

class CanHandler:
    def __init__(self, channel="can0", bustype="socketcan"):
        self.bus = can.interface.Bus(channel=channel, bustype=bustype)

    def __call__(self, msg):
        # 여기에서 수신된 메시지를 처리합니다.
        print(f"Received CAN message: {msg}")
        # 필요에 따라 메시지를 파싱하거나 추가 작업을 수행할 수 있습니다.

    def send_message(self, can_id, data):
        try:
            message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self.bus.send(message)
            print(f"Sent CAN message: ID={can_id}, data={data}")
        except can.CanError as e:
            print(f"Error sending CAN message: {e}")
            raise

    def receive_message(self, timeout=1.0):
        try:
            response = self.bus.recv(timeout)  # 타임아웃 설정
            if response:
                print(f"Received CAN message: {response}")
                return response
            else:
                print("No response received within the timeout period.")
                return None
        except Exception as e:
            print(f"Error receiving CAN message: {e}")

    def close(self):
        try:
            self.bus.shutdown()  # 버스를 종료하고 리소스를 해제
            print("CAN bus shut down properly.")
        except Exception as e:
            print(f"Error during CAN bus shutdown: {e}")
