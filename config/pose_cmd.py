from dataclasses import dataclass, field

@dataclass
class PoseCommand:
    fl_coord: list = field(default_factory=lambda: [0.0, 0.0, 0.0])  # 앞 왼쪽 다리
    fr_coord: list = field(default_factory=lambda: [0.0, 0.0, 0.0])  # 앞 오른쪽 다리
    rl_coord: list = field(default_factory=lambda: [0.0, 0.0, 0.0])  # 뒤 왼쪽 다리
    rr_coord: list = field(default_factory=lambda: [0.0, 0.0, 0.0])  # 뒤 오른쪽 다리

    def update_pose(self, leg_name: str, coords: list):
        """지정된 다리의 좌표를 업데이트"""
        if leg_name == "fl_coord":
            self.fl_coord = coords
        elif leg_name == "fr_coord":
            self.fr_coord = coords
        elif leg_name == "rl_coord":
            self.rl_coord = coords
        elif leg_name == "rr_coord":
            self.rr_coord = coords
        else:
            print(f"Invalid leg name: {leg_name}")

    def get_pose(self):
        """현재 포즈 가져오기"""
        return {
            "fl_coord": self.fl_coord,
            "fr_coord": self.fr_coord,
            "rl_coord": self.rl_coord,
            "rr_coord": self.rr_coord
        }

class PoseManager:
    def __init__(self):
        self.pose_cmd = PoseCommand()

    def update_pose(self, leg_name: str, coords: list):
        """포즈 갱신"""
        self.pose_cmd.update_pose(leg_name, coords)
        print(self.pose_cmd.get_pose())
