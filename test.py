import numpy as np
import matplotlib.pyplot as plt

# 시간 설정
T = 2  # 한 사이클 주기 (초)
dt = 0.01  # 시간 간격
t = np.arange(0, T, dt)  # 시간 벡터

# 보행 주파수와 진폭 설정
stride_length = 50  # 한 사이클당 이동 거리 (mm)
step_height = 20  # 들어올리는 높이 (mm)

# X 방향 이동 (사인 함수 사용)
x = stride_length * np.sin(2 * np.pi * t / T)

# Z 방향 이동 초기화 (모든 값 0)
z = np.zeros_like(x)

# x가 증가하는 구간 (최솟값 → 최댓값)에서 z를 상승 및 하강
rising_phase = x >= np.roll(x, 1)  # 현재 x가 이전 값보다 크면 증가 구간

# 증가 구간에서 z를 부드럽게 올렸다가 내리기 (코사인 곡선 사용)
t_rising = np.linspace(0, np.pi, np.sum(rising_phase))
z[rising_phase] = step_height * (1 - np.cos(t_rising)) / 2

# 그래프 출력
plt.figure(figsize=(8, 4))
plt.plot(x, z, label="Foot Trajectory", color="b")
plt.xlabel("X Position (mm)")
plt.ylabel("Z Position (mm)")
plt.title("Quadruped Foot Trajectory (Final Version)")
plt.legend()
plt.grid()
plt.show()

