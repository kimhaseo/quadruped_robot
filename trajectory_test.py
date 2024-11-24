import numpy as np

def calculate_trajectory(self, foot_direction, time_values):
    step_length = 50
    step_high = 30

    # x, y, z 값 초기화
    x = np.zeros_like(time_values)
    y = np.zeros_like(time_values)
    z = np.zeros_like(time_values)

    if foot_direction == "forward":
        for i, t in enumerate(time_values):
            # 첫 번째 구간: x 0에서 절반 감소, z = 0
            if 0 <= t < 0.5:
                x[i] = 0 - (step_length / 2) * (t / 0.5)  # x는 0에서 -step_length/2까지 감소
                z[i] = 0  # z는 0
            # 두 번째 구간: x 절반 증가, z = step_high 증가
            elif 0.5 <= t < 1.0:
                x[i] = -step_length / 2 + (step_length / 2) * (2 * (t - 0.5) / 0.5)  # x는 -step_length/2에서 0까지 증가
                z[i] = step_high * (2 * (t - 0.5) / 0.5)  # z는 0에서 step_high까지 증가
            # 세 번째 구간: x 절반 더 증가, z = step_high에서 0으로 서서히 감소
            elif 1.0 <= t < 1.5:
                x[i] = 0 + (step_length / 2) * ((t - 1.0) / 0.5)  # x는 0에서 step_length/2까지 증가
                z[i] = step_high * (1 - 2 * (t - 1.0) / 0.5)  # z는 step_high에서 0으로 선형 감소
            # 네 번째 구간: x 절반 감소, z = 0
            else:
                x[i] = step_length / 2 - (step_length / 2) * (2 * (t - 1.5) / 0.5)  # x는 step_length/2에서 0까지 감소
                z[i] = 0  # z는 0

            # y 값은 변하지 않으므로 0으로 설정
            y[i] = 0

    return np.array(x), np.array(y), np.array(z)

# 예시 실행
time_values = np.linspace(0, 2, 50)  # 0부터 2까지 30개의 시간 값
foot_direction = "forward"  # 발이 "forward" 방향으로 이동한다고 설정

# 경로 계산
x_vals, y_vals, z_vals = calculate_trajectory(None, foot_direction, time_values)

# 출력
for t, x, y, z in zip(time_values, x_vals, y_vals, z_vals):
    print(f"time: {t:.2f}, x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")
