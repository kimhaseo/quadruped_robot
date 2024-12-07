import numpy as np
from config import config,pose_cmd
from solver import inverse

class TrajectoryGenerator:
    def __init__(self):
        """
        Gait Pattern Generator 초기화.

        :param step_height: 발의 높이 (mm)
        :param speed: 이동 속도 (mm/s)
        :param height: 기본 발 위치 높이 (mm)
        """
        self.resolution = config.leg_resolution

    def move_foot_trajectory(self, time, start_step, foot_direction ,speed, step_height):
        """
        한쪽 발의 궤적 계산.
        :param time: 시간 배열
        :param is_left: 왼쪽 발인지 여부 (True/False)
        :return: x, z 좌표 배열
        """

        self.step_length = speed * 0.75
        self.step_height = step_height

        if start_step == 1 :

            if foot_direction == "forward":
                # x 좌표 설정
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
                y = np.zeros_like(time)
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

        elif start_step == 0 :

            if foot_direction == "forward":
                x = np.where(
                    (0 <= time) & (time < 0.5),
                    self.step_length * (time / 0.5),  # 0에서 step_length까지 증가
                    np.where(
                        (0.5 <= time) & (time < 1.0),
                        self.step_length * (1 - (time - 0.5) / 0.5),  # step_length에서 0으로 감소
                        np.where(
                            (1.0 <= time) & (time < 1.5),
                            -self.step_length * ((time - 1.0) / 0.5),  # 0에서 -step_length까지 감소
                            -self.step_length * (1 - (time - 1.5) / 0.5)  # -step_length에서 0으로 증가
                        )
                    )
                )
                y = np.zeros_like(time)
                z = np.where(
                    (0 <= time) & (time < 0.5),
                    self.step_height * (1 - time / 0.5),  # step_height에서 0까지 선형적으로 감소
                    np.where(
                        (0.5 <= time) & (time < 1.0),
                        0,  # z는 0으로 유지
                        np.where(
                            (1.0 <= time) & (time < 1.5),
                            0,  # z는 0으로 유지
                            self.step_height * ((time - 1.5) / 0.5)  # 0에서 step_height까지 증가
                        )
                    )
                )


        return np.stack((x, y, z), axis=1)

    def generate_move_trajectory(self, speed, step_hight, robot_motion):

        self.speed = speed
        self.step_hight = step_hight
        self.robot_motion = robot_motion
        time = np.linspace(0, 2, self.resolution)

        if robot_motion == "forward":
            foot_direction = ["forward","forward","forward","forward"]

        elif robot_motion == "backward":
            foot_direction = ["backward","backward","backward","backward"]

        elif robot_motion == "left":
            foot_direction = ["left","left","left","left"]

        elif robot_motion == "right":
            foot_direction = ["right","right","right","right"]

        elif robot_motion == "left_turn":
            foot_direction = ["backward","forward","backward","forward"]

        elif robot_motion == "right_turn":
            foot_direction = ["forward","backward","forward","backward"]


        fl_coords  = self.move_foot_trajectory(time,1,foot_direction[0], self.speed , self.step_hight)
        fr_coords  = self.move_foot_trajectory(time,0,foot_direction[1], self.speed , self.step_hight)
        rl_coords = self.move_foot_trajectory(time,0,foot_direction[2], self.speed , self.step_hight)
        rr_coords  = self.move_foot_trajectory(time,1,foot_direction[3], self.speed , self.step_hight)

        return (fl_coords, fr_coords, rl_coords, rr_coords)

    def pose_foot_trajectory(self, goal_point, start_point):

        goal_point = np.array(goal_point)
        start_point = np.array(start_point)
        delta = goal_point - start_point
        # 분해능에 따라 이동 단계 계산goal_point
        steps = np.linspace(0, 1,self.resolution)
        # 경로 생성
        foot_trajectory = [start_point + step * delta for step in steps]
        return foot_trajectory

    def generate_pose_trajectory(self,target_pose,current_pose):

        fl_foot_coords = self.pose_foot_trajectory(target_pose['fl_foot'],current_pose['fl_foot'])
        fr_foot_coords = self.pose_foot_trajectory(target_pose['fr_foot'],current_pose['fr_foot'])
        rl_foot_coords = self.pose_foot_trajectory(target_pose['rl_foot'],current_pose['rl_foot'])
        rr_foot_coords = self.pose_foot_trajectory(target_pose['rr_foot'],current_pose['rr_foot'])
        
        return fl_foot_coords,fr_foot_coords,rl_foot_coords,rr_foot_coords



if __name__ == "__main__":

    tg=TrajectoryGenerator()
    ik=inverse.Kinematics()
    trajetory = tg.generate_move_trajectory(60,20,"forward")
    print(trajetory)
    # target_pose = config.init_pose
    # pc = pose_cmd.PoseCommand()
    # current_pose =pc.get_pose()
    # pose_trajectory = tg.generate_pose_trajectory(target_pose,current_pose)

    # start = np.array([0.0, 0.0, 0.0])  # 초기 포즈 (x, y, z)
    # goal = np.array([0.5, 0.2, 0.3])  # 목표 포즈 (x, y, z)
    #
    # foot_pose= ik.calculate_foot_position_with_orientation(10,0,0)
    # fl_trajectory = tg.pose_foot_trajectory(foot_pose[0], config.start_pose["fl_coord"])
    # print(fl_trajectory[0])
    # for i in range(200):
    #
    #     fl_theta1,fl_theta2,fl_theta3 = ik.calculate_joint_angle(False,fl_trajectory[i][0],fl_trajectory[i][1],fl_trajectory[i][2])
    #     fr_theta1,fr_theta2,fr_theta3 = ik.calculate_joint_angle(False, fl_trajectory[i][0], fl_trajectory[i][1],fl_trajectory[i][2])
    #     rl_theta1,rl_theta2,rl_theta3 = ik.calculate_joint_angle(False, fl_trajectory[i][0], fl_trajectory[i][1],fl_trajectory[i][2])
    #     rr_theta1,rr_theta2,rr_theta3 = ik.calculate_joint_angle(False, fl_trajectory[i][0], fl_trajectory[i][1],fl_trajectory[i][2])
    #
    #
    #

