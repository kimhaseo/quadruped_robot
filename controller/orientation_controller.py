from solver.inverse import Kinematics


class RobotControl:
    def __init__(self, robot_orientation):
        self.robot_orientation = robot_orientation
        self.kinematics = Kinematics()

    def orientation_control(self):
        # 주어진 오리엔테이션에 맞는 발 위치 계산
        calc_foot_pose = self.kinematics.calculate_foot_position_with_orientation(*self.robot_orientation)

        # 발 위치에서 힙 위치를 빼서 제어할 발 위치 계산
        orientation_foot_pose = calc_foot_pose - self.kinematics.hip_positions

        return orientation_foot_pose


if __name__ == "__main__":
    # 오리엔테이션을 정의하고 로봇 제어 클래스 인스턴스를 생성
    robot_orientation = [0, 10, 0]
    robot_control = RobotControl(robot_orientation)

    # 발 위치 제어 결과 출력
    cal_foot_pose = robot_control.orientation_control()
    print(cal_foot_pose)