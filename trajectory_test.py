import numpy as np

# 설정
self = type('dummy', (), {'step_length': 50, 'step_height': 20})()  # step_length와 step_height 설정
time = np.linspace(0, 2, 21)  # 0초부터 2초까지 0.1초 간격

# x, z 좌표 계산
x = np.where(
    (0 <= time) & (time < 0.5),
    -self.step_length * (time / 0.5),  # 0에서 -step_length까지 감소
    np.where(
        (0.5 <= time) & (time < 1.0),
        -self.step_length * (1 - (time - 0.5) / 0.5),  # -step_length에서 0으로 증가
        np.where(
            (1.0 <= time) & (time < 1.5),
            self.step_length * ((time - 1.0) / 0.5),  # 0에서 step_length까지 증가
            self.step_length * (1 - (time - 1.5) / 0.5)  # step_length에서 0으로 감소
        )
    )
)

z = np.where(
    (0 <= time) & (time < 0.5),
    0,  # z는 0으로 유지
    np.where(
        (0.5 <= time) & (time < 1.0),
        self.step_height * ((time - 0.5) / 0.5),  # 0에서 step_height까지 증가
        np.where(
            (1.0 <= time) & (time < 1.5),
            self.step_height * (1 - (time - 1.0) / 0.5),  # step_height에서 0까지 감소
            0  # z는 0으로 유지
        )
    )
)

# 시간별 x, z 좌표 출력
for t, x_val, z_val in zip(time, x, z):
    print(f"time = {t:.2f}, x = {x_val:.2f}, z = {z_val:.2f}")