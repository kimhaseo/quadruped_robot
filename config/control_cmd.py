from dataclasses import dataclass, field
from typing import List
import threading  # 🔥 Lock 사용

@dataclass
class ControlCommand:
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    lock: threading.Lock = field(default_factory=threading.Lock)  # 🔥 Lock 추가

    def update_(self, linear_delta: float = 0.0, angular_delta: float = 0.0, orientation_delta: List[float] = None):
        """ 입력된 값만큼 현재 속도와 자세를 업데이트하는 함수 (스레드 안전) """
        with self.lock:  # 🔥 Lock으로 보호
            # 선속도와 각속도 업데이트
            self.linear_velocity += linear_delta
            self.angular_velocity += angular_delta

            # 자세 업데이트
            if orientation_delta:
                self.orientation = [o + d for o, d in zip(self.orientation, orientation_delta)]

            # 값 제한 (예제)
            self.linear_velocity = max(min(self.linear_velocity, 2.0), -2.0)
            self.angular_velocity = max(min(self.angular_velocity, 1.0), -1.0)

            # print(f"Updated Command: {self}")

    def get_control_cmd(self):
        """ 현재 속도 및 자세 값을 반환 (스레드 안전) """
        with self.lock:  # 🔥 Lock으로 보호
            return {
                "linear_velocity": self.linear_velocity,
                "angular_velocity": self.angular_velocity,
                "orientation": self.orientation,
            }


# 테스트 코드 (멀티스레드 환경 테스트)
if __name__ == "__main__":
    import threading

    cmd = ControlCommand()

    def thread_task():
        for _ in range(5):
            cmd.update_(linear_delta=0.1, angular_delta=0.05, orientation_delta=[0.1, 0.0, -0.05])

    # 🔥 두 개의 스레드에서 동시에 업데이트 수행
    t1 = threading.Thread(target=thread_task)
    t2 = threading.Thread(target=thread_task)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    print("최종 상태:", cmd.get_control_cmd())
