import cv2
from ultralytics import YOLO
import time

# YOLOv8 모델 로드 (Open Images V7로 학습된 모델)
model = YOLO('yolov8n-oiv7.pt')

# 카메라 설정 (기본 카메라 사용, 카메라 ID 0)
cap = cv2.VideoCapture(0)

# 해상도 설정: 640x480
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# 카메라에서 프레임을 가져와 처리하는 루프
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # YOLO 모델로 프레임 예측
    results = model(frame)

    # 결과 이미지 얻기 (예측된 이미지)
    annotated_frame = results[0].plot()  # 예측 결과를 이미지에 그리기
    # 결과를 실시간으로 창에 표시
    cv2.imshow('Real-time Object Detection', annotated_frame)

    # 'q' 키를 눌러 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 카메라 종료 및 창 닫기
cap.release()
cv2.destroyAllWindows()
