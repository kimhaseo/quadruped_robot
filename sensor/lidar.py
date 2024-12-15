import pyrealsense2 as rs
import numpy as np


def get_depth_data_in_roi(depth_frame, roi, depth_scale):
    """
    지정된 ROI 범위에서 Depth 데이터를 가져옵니다.
    - depth_frame: RealSense Depth Frame
    - roi: ROI 범위 (dict: {'x': int, 'y': int, 'width': int, 'height': int})
    - depth_scale: Depth 값에 적용할 스케일 (단위: m)

    반환값:
    - ROI 영역의 Depth 데이터 (단위: m)
    """
    x, y, width, height = roi['x'], roi['y'], roi['width'], roi['height']
    depth_data = np.asanyarray(depth_frame.get_data())
    roi_data = depth_data[y:y + height, x:x + width]  # ROI에서 데이터 슬라이싱
    return roi_data * depth_scale  # 스케일링 후 반환


# 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # D405 기본 해상도

# Depth 스케일 가져오기
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
depth_sensor = device.first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()  # Depth 값을 미터로 변환하는 스케일

# 파이프라인 시작
pipeline.start(config)

try:
    while True:
        # 프레임 가져오기
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        # ROI 설정 (예: 이미지 중앙 하단 부분)
        roi = {'x': 200, 'y': 300, 'width': 240, 'height': 100}

        # ROI에서 Depth 데이터 읽기
        roi_depth_data = get_depth_data_in_roi(depth_frame, roi, depth_scale)

        # ROI 내 최소값, 평균값 계산
        min_depth = np.min(roi_depth_data)
        mean_depth = np.mean(roi_depth_data)

        print(f"ROI 최소 깊이: {min_depth:.3f} m, 평균 깊이: {mean_depth:.3f} m")

except KeyboardInterrupt:
    print("프로그램 종료 중...")

finally:
    pipeline.stop()