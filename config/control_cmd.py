from dataclasses import dataclass, field
from typing import List


@dataclass
class ControlCommand:
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    def update_(self, linear_delta: float = 0.0, angular_delta: float = 0.0, orientation_delta: List[float] = None):
        """ 입력된 값만큼 현재 속도와 자세를 업데이트하는 함수 """

        # 선속도와 각속도 업데이트
        self.linear_velocity += linear_delta
        self.angular_velocity += angular_delta

        # 자세 업데이트
        if orientation_delta:
            self.orientation = [o + d for o, d in zip(self.orientation, orientation_delta)]

        # 값 제한 (예제)
        self.linear_velocity = max(min(self.linear_velocity, 2.0), -2.0)
        self.angular_velocity = max(min(self.angular_velocity, 1.0), -1.0)

        print(f"Updated Command: {self}")

    def get_control_cmd(self):
        """ 현재 속도 및 자세 값을 반환 """
        return {
            "linear_velocity": self.linear_velocity,
            "angular_velocity": self.angular_velocity,
            "orientation": self.orientation,
        }


# 테스트 코드
if __name__ == "__main__":
    cmd = ControlCommand()
    cmd.update_(linear_delta=0.5, angular_delta=0.2, orientation_delta=[1.0, 0.0, -0.5])
    print(cmd.get_control_cmd())  # 현재 상태 출력

    cmd.update_(linear_delta=-0.3, angular_delta=-0.1, orientation_delta=[-0.2, 0.3, 0.1])
    print(cmd.get_control_cmd())  # 업데이트 후 상태 출력
