import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dataclasses import dataclass, field
from config.config import start_pose
import threading

@dataclass
class PoseCommand:
    fl_foot: list = field(default_factory=lambda: [start_pose["fl_foot"][0], start_pose["fl_foot"][1], start_pose["fl_foot"][2]])  # 앞 왼쪽 다리
    fr_foot: list = field(default_factory=lambda: [start_pose["fr_foot"][0], start_pose["fr_foot"][1], start_pose["fr_foot"][2]])  # 앞 오른쪽 다리
    rl_foot: list = field(default_factory=lambda: [start_pose["rl_foot"][0], start_pose["rl_foot"][1], start_pose["rl_foot"][2]])  # 뒤 왼쪽 다리
    rr_foot: list = field(default_factory=lambda: [start_pose["rr_foot"][0], start_pose["rr_foot"][1], start_pose["rr_foot"][2]])  # 뒤 오른쪽 다리
    body_orientation: list = field(default_factory=lambda: [0.0, 0.0, 0.0])  # Roll, Pitch, Yaw
    body_vel:list = field(default_factory=lambda: [0.0, 0.0])
    body_position:list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    lock: threading.Lock = field(default_factory=threading.Lock)  # Lock 추가

    def update_pose(self, name: str, coords: list):
        """지정된 다리의 좌표를 업데이트 (Lock 사용)"""
        with self.lock:  # Lock으로 보호
            if name == "fl_foot":
                self.fl_foot = coords
            elif name == "fr_foot":
                self.fr_foot = coords
            elif name == "rl_foot":
                self.rl_foot = coords
            elif name == "rr_foot":
                self.rr_foot = coords
            elif name =="body_orientation":
                self.body_orientation = coords
            elif name =="body_vel":
                self.body_vel = coords
            elif name =="body_position":
                self.body_position = coords
            else:
                print(f"Invalid leg name: {name}")

    def update_orientation(self, orientation: list):
        """로봇의 몸체 자세(Orientation)를 업데이트 (Lock 사용)"""
        with self.lock:  # Lock으로 보호
            if len(orientation) == 3:
                self.body_orientation = orientation
            else:
                print("Orientation must be a list of 3 values [Roll, Pitch, Yaw].")

    def get_pose(self):
        """현재 포즈 가져오기"""
        with self.lock:  # Lock으로 보호
            data = {
                "fl_foot": self.fl_foot,
                "fr_foot": self.fr_foot,
                "rl_foot": self.rl_foot,
                "rr_foot": self.rr_foot,
                "body_orientation": self.body_orientation,
                "body_vel":self.body_vel,
                "body_position":self.body_position,
            }
            rounded_data = {key: [round(coord, 1) for coord in coords] for key, coords in data.items()}
            return rounded_data
