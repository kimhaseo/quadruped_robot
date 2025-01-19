# depth_sensor.py
import pyrealsense2 as rs
import numpy as np

class DepthSensor:
    def __init__(self, width=640, height=480, depth_stream_format=rs.format.z16, fps=30):
        # 파이프라인 설정
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, width, height, depth_stream_format, fps)

        # Depth 스케일 가져오기
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.depth_sensor = self.device.first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()  # Depth 값을 미터로 변환하는 스케일

        # 파이프라인 시작
        self.pipeline.start(self.config)

    def get_depth_data_in_roi(self, depth_frame, roi):
        """
        지정된 ROI 범위에서 Depth 데이터를 가져옵니다.
        - depth_frame: RealSense Depth Frame
        - roi: ROI 범위 (dict: {'x': int, 'y': int, 'width': int, 'height': int})

        반환값:
        - ROI 영역의 Depth 데이터 (단위: m)
        """
        x, y, width, height = roi['x'], roi['y'], roi['width'], roi['height']
        depth_data = np.asanyarray(depth_frame.get_data())
        roi_data = depth_data[y:y + height, x:x + width]  # ROI에서 데이터 슬라이싱
        return roi_data * self.depth_scale  # 스케일링 후 반환

    def get_depth_stats_in_roi(self, roi):
        """
        ROI 내에서 Depth 값의 최소값과 평균값을 반환합니다.
        """
        # 프레임 가져오기
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None, None

        # ROI에서 Depth 데이터 읽기
        roi_depth_data = self.get_depth_data_in_roi(depth_frame, roi)

        # ROI 내 최소값, 평균값 계산
        min_depth = np.min(roi_depth_data)
        mean_depth = np.mean(roi_depth_data)

        return min_depth, mean_depth

    def stop(self):
        """파이프라인 종료"""
        self.pipeline.stop()
