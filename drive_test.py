from solver.trajectory import GaitPatternGenerator
from solver.inverse import Kinematics
from config.config import body_dimensions




robot_direction = "forward"
robot_speed = 30 ## mm/s
robot_orientation = [0,10,0]





if __name__ == "__main__":

   kinematics = Kinematics()
   calc_foot_pose = kinematics.calculate_foot_position_with_orientation(*robot_orientation)
   control_pose = calc_foot_pose-kinematics.hip_positions
   print(control_pose[0])
   left_foot_pose = kinematics.calculate_joint_angle(*control_pose[0])
   print(left_foot_pose)