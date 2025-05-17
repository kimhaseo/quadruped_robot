import pygame

# 초기화
pygame.init()
pygame.joystick.init()

# 첫 번째 조이스틱 연결
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"조이스틱 이름: {joystick.get_name()}")

# 메인 루프
while True:
    pygame.event.pump()  # 입력 갱신

    axis_0 = joystick.get_axis(0)  # 왼쪽 스틱 X축
    axis_1 = joystick.get_axis(1)  # 왼쪽 스틱 Y축
    button_0 = joystick.get_button(0)  # 버튼 0

    print(f"X축: {axis_0:.2f}, Y축: {axis_1:.2f}, 버튼0: {button_0}")

    # 예: 왼쪽 스틱으로 로봇 이동 방향 제어
    # 여기에 로봇 제어 코드 추가

    pygame.time.wait(100)
