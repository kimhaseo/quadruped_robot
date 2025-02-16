import math
from idlelib.run import handle_tk_events


class CosineLawCalculator:
    def __init__(self, a: float, b: float, c: float, d: float):
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    def cosine_law(self, a: float, b: float, C: float) -> float:
        """코사인 법칙을 이용하여 세 번째 변의 길이를 계산하는 함수"""
        C_rad = math.radians(C)  # 각도를 라디안으로 변환
        c = math.sqrt(a**2 + b**2 - 2 * a * b * math.cos(C_rad))  # 코사인 법칙 적용
        return c

    def find_angle(self, a: float, b: float, c: float) -> float:
        """코사인 법칙을 사용하여 두 변 사이의 각도를 계산하는 함수"""
        cos_angle = (a**2 + b**2 - c**2) / (2 * a * b)
        angle_rad = math.acos(cos_angle)  # 라디안 단위
        angle_deg = math.degrees(angle_rad)  # 도 단위 변환
        return angle_deg

    def calculate_angle(self, theta: float) -> float:
        """주어진 theta에 대한 angle을 계산하는 함수"""
        theta = 180-theta
        line = self.cosine_law(self.a, self.d, theta)
        theta_a = self.find_angle(line, self.a, self.d)
        theta_b = self.find_angle(line, self.b, self.c)
        angle = theta_a + theta_b
        return angle

# 사용 예시
calculator = CosineLawCalculator(a=200, b=30, c=190, d=15)
theta = 90
angle = calculator.calculate_angle(theta)
print(angle)
