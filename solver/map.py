import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# 랜덤 LiDAR 데이터 생성 함수 (실시간으로 데이터를 넘겨줌)
def generate_lidar_data(num_points, x_range, y_range):
    x = np.random.uniform(x_range[0], x_range[1], num_points)  # x 좌표
    y = np.random.uniform(y_range[0], y_range[1], num_points)  # y 좌표
    return x, y


# 실시간 2D 지도 업데이트 함수
def update_map(frame, scat, num_points, x_range, y_range):
    # 새로운 랜덤 LiDAR 데이터 생성
    x, y = generate_lidar_data(num_points, x_range, y_range)

    # 기존 데이터에 새로운 데이터를 추가하여 업데이트
    scat.set_offsets(np.c_[x, y])

    return scat,


# 설정
num_points = 100  # 한번에 생성할 랜덤 점의 수
x_range = (-1000, 1000)
y_range = (-1000, 1000)

# 초기 설정: 빈 플롯
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(x_range)
ax.set_ylim(y_range)
ax.set_title("Real-time 2D LiDAR Map")
ax.set_xlabel("X (meters)")
ax.set_ylabel("Y (meters)")
ax.grid(True)

# 초기 데이터 포인트
x, y = generate_lidar_data(num_points, x_range, y_range)
scat = ax.scatter(x, y, s=10, c='blue', alpha=0.6)

# FuncAnimation을 사용하여 실시간 업데이트
ani = FuncAnimation(fig, update_map, fargs=(scat, num_points, x_range, y_range),
                    frames=200, interval=100, blit=False)

plt.show()

