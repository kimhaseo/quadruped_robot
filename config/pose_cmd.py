import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dataclasses import dataclass, field
from config.config import start_pose

@dataclass
class PoseCommand:
    fl_foot: list = field(default_factory=lambda: [start_pose["fl_foot"][0], start_pose["fl_foot"][1], start_pose["fl_foot"][2]])  # 앞 왼쪽 다리
    fr_foot: list = field(default_factory=lambda: [start_pose["fr_foot"][0], start_pose["fr_foot"][1], start_pose["fr_foot"][2]])  # 앞 오른쪽 다리
    rl_foot: list = field(default_factory=lambda: [start_pose["rl_foot"][0], start_pose["rl_foot"][1], start_pose["rl_foot"][2]])  # 뒤 왼쪽 다리
    rr_foot: list = field(default_factory=lambda: [start_pose["rr_foot"][0], start_pose["rr_foot"][1], start_pose["rr_foot"][2]])  # 뒤 오른쪽 다리

    def update_pose(self, leg_name: str, coords: list):
        """지정된 다리의 좌표를 업데이트"""
        if leg_name == "fl_foot":
            self.fl_foot = coords
        elif leg_name == "fr_foot":
            self.fr_foot = coords
        elif leg_name == "rl_foot":
            self.rl_foot = coords
        elif leg_name == "rr_foot":
            self.rr_foot = coords
        else:
            print(f"Invalid leg name: {leg_name}")

    def get_pose(self):
        """현재 포즈 가져오기"""
        data=  {
            "fl_foot": self.fl_foot,
            "fr_foot": self.fr_foot,
            "rl_foot": self.rl_foot,
            "rr_foot": self.rr_foot
        }

        rounded_data = {key: [round(coord, 1) for coord in coords] for key, coords in data.items()}
        return rounded_data

class PoseManager:
    def __init__(self):
        self.pose_cmd = PoseCommand()

    def update_pose(self, leg_name: str, coords: list):
        """포즈 갱신"""
        self.pose_cmd.update_pose(leg_name, coords)
        print(self.pose_cmd.get_pose())
