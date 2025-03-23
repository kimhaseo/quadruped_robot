import numpy as np
import threading
import time
from manager.pose_manager import pose_cmd
from config.config import hip_pose

class VelocityEstimator:
    def __init__(self, dt=0.01):
        self.dt = dt  # 샘플링 시간 간격
        self.running = False
        self.v_com = np.zeros(3)  # 로봇의 선속도 (x, y, z)
        self.omega = np.zeros(3)  # 로봇의 각속도 (x, y, z)
        self.lock = threading.Lock()
        self.prev_time = time.time()  # 이전 시간 저장

    def get_foot_positions(self):
        """ 현재 발 위치를 가져옴 """
        pose = pose_cmd.get_pose()
        pose = pose

        return np.array([
            np.array(pose["fl_foot"]) - np.array(hip_pose["fl_hip"]),
            np.array(pose["fr_foot"]) - np.array(hip_pose["fr_hip"]),
            np.array(pose["rl_foot"]) - np.array(hip_pose["rl_hip"]),
            np.array(pose["rr_foot"]) - np.array(hip_pose["rr_hip"])
        ])

    def compute_velocity(self):
        """ 선속도 및 각속도 계산 """
        p_prev = self.get_foot_positions()
        time.sleep(self.dt)

        while self.running:
            current_time = time.time()
            dt_actual = current_time - self.prev_time

            if dt_actual >= self.dt:
                p_curr = self.get_foot_positions()

                # 각 발의 속도 계산
                v_legs = (p_curr - p_prev) / dt_actual

                # 로봇의 중심 위치 (COM)
                p_com = np.mean(p_curr, axis=0)

                # z 값은 각 발의 최솟값으로 고정
                z = np.min(p_curr[:, 2])  # 각 발의 z값 중 최솟값을 사용

                # x 값을 각 발의 절대값 평균으로 계산 후 4로 나누기
                x = np.mean(np.abs(p_curr[:, 0])) / 4  # 각 발의 x값 절대값 평균으로 계산 후 4로 나누기
                y = np.mean(p_curr[:, 1]) / 4  # y값은 그대로 평균 후 4로 나누기

                # 로봇 중심 속도 (선속도)
                v_com = np.array([x, y, z])

                # 각 발의 상대 위치 (로봇 중심 기준)
                r_legs = p_curr - p_com

                # 속도 차이 (각 발 속도 - 로봇 선속도)
                v_diff = v_legs - v_com

                # 각속도 계산을 위한 행렬 설정
                cross_matrix = np.array([np.cross(np.eye(3), r) for r in r_legs])
                cross_matrix = cross_matrix.reshape(-1, 3)

                # 최소자승법(Least Squares)으로 각속도 추정
                omega, _, _, _ = np.linalg.lstsq(cross_matrix, v_diff.flatten(), rcond=None)

                # 스레드 안전한 데이터 업데이트
                with self.lock:
                    self.v_com = v_com
                    self.omega = omega

                # 이전 값을 현재 값으로 갱신
                p_prev = p_curr.copy()
                self.prev_time = current_time  # 시간 갱신

            time.sleep(0.001)  # CPU 부하를 줄이기 위한 짧은 대기

    def start(self):
        """ 속도 추정 스레드 시작 """
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.compute_velocity, daemon=True)
            self.thread.start()

    def stop(self):
        """ 속도 추정 스레드 종료 """
        self.running = False
        self.thread.join()

    def get_velocity(self):
        """ 현재 속도 반환 (스레드 안전) """
        with self.lock:
            return self.v_com.copy(), self.omega.copy()
