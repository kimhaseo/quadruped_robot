import cv2
import torch

# YOLO 모델 로드 (PyTorch Hub 사용)
def load_yolo_model():
    print("Loading YOLO model...")
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # yolov5s 모델 로드
    return model

# YOLO 모델로 추론 수행
def run_inference(model, frame):
    # OpenCV 이미지를 PyTorch 텐서로 변환
    results = model(frame)
    return results

# 결과 그리기 함수
def draw_results(frame, results):
    # results.xyxy[0] : [x1, y1, x2, y2, confidence, class] 형식
    for *box, conf, cls in results.xyxy[0]:
        x1, y1, x2, y2 = map(int, box)
        label = f"{results.names[int(cls)]} {conf:.2f}"
        # 경계 상자 그리기
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # 텍스트 그리기
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return frame

# USB 카메라 설정
def main():
    camera_index = 0  # 기본 카메라 인덱스
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print("Error: Could not open video source.")
        return

    model = load_yolo_model()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # YOLO 모델로 추론 수행
        results = run_inference(model, frame)

        # 결과를 프레임에 표시
        frame = draw_results(frame, results)

        # 결과 프레임 출력
        cv2.imshow('YOLO Detection', frame)

        # 종료 조건: ESC 키
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


