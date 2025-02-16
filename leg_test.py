import math

from scipy.spatial.distance import cosine

a = 200
b = 30
c = 200
d = 15
theta = 90


def cosine_law(a: float, b: float, C: float) -> float:
    """코사인 법칙을 이용하여 세 번째 변의 길이를 계산하는 함수
    a, b: 주어진 두 변의 길이
    C: 두 변 사이의 각 (도 단위)
    """
    C_rad = math.radians(C)  # 각도를 라디안으로 변환
    c = math.sqrt(a**2 + b**2 - 2 * a * b * math.cos(C_rad))  # 코사인 법칙 적용
    return c

def find_angle(a: float, b: float, c: float) -> float:
    """
    코사인 법칙을 사용하여 첫 번째 변(a)과 두 번째 변(b) 사이의 각도를 계산하는 함수
    a, b, c: 삼각형의 세 변의 길이
    반환값: 각도 (도 단위)
    """
    cos_angle = (a**2 + b**2 - c**2) / (2 * a * b)
    angle_rad = math.acos(cos_angle)  # 라디안 단위
    angle_deg = math.degrees(angle_rad)  # 도 단위 변환
    return angle_deg

line = cosine_law(a,b,theta)
theta_a = find_angle(line,a,b)
theta_b = find_angle(line,d,c)
angle = 180-(theta_a+theta_b)

print(angle)
