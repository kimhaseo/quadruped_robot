from solver.trajectory import GaitPatternGenerator

robot_speed = 30 #mm/s
robot_step_hight = 30
robot_motion = "forward"
robot_oriention = robot_orientation = [0, 10, 0]


def driving_contoller():

    pattern_generator = GaitPatternGenerator()
    test = pattern_generator.generate_crawl_gait_pattern(robot_motion)



if __name__ == "__main__":
    driving_contoller()