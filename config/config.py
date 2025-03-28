
leg_length = {
    "L1": 89,
    "L2": 170,
    "L3": 200
}

body_dimensions = {
    "length": 400,  # mm
    "width": 110,   # mm
    "height": 270   # mm
}

leg_resolution = 200

hip_pose = {
    "fl_hip": [body_dimensions["length"]/2, -body_dimensions["width"]/2, 0],
    "fr_hip": [body_dimensions["length"]/2, body_dimensions["width"]/2, 0],
    "rl_hip": [-body_dimensions["length"]/2, -body_dimensions["width"]/2, 0],
    "rr_hip": [-body_dimensions["length"]/2, body_dimensions["width"]/2, 0],
}

init_pose = {
    "fl_foot": [hip_pose["fl_hip"][0], -leg_length["L1"] + hip_pose["fl_hip"][1], -body_dimensions["height"]],
    "fr_foot": [hip_pose["fr_hip"][0], leg_length["L1"] + hip_pose["fr_hip"][1], -body_dimensions["height"]],
    "rl_foot": [hip_pose["rl_hip"][0], -leg_length["L1"] + hip_pose["rl_hip"][1], -body_dimensions["height"]],
    "rr_foot": [hip_pose["rr_hip"][0], leg_length["L1"] + hip_pose["rr_hip"][1], -body_dimensions["height"]],
}

start_pose = {
    "fl_foot": [hip_pose["fl_hip"][0], -leg_length["L1"] + hip_pose["fl_hip"][1], -270],
    "fr_foot": [hip_pose["fr_hip"][0], leg_length["L1"] + hip_pose["fr_hip"][1], -270],
    "rl_foot": [hip_pose["rl_hip"][0], -leg_length["L1"] + hip_pose["rl_hip"][1], -270],
    "rr_foot": [hip_pose["rr_hip"][0], leg_length["L1"] + hip_pose["rr_hip"][1], -270],
}

down_pose = {
    "fl_foot": [hip_pose["fl_hip"][0], -leg_length["L1"] + hip_pose["fl_hip"][1], -150],
    "fr_foot": [hip_pose["fr_hip"][0], leg_length["L1"] + hip_pose["fr_hip"][1], -150],
    "rl_foot": [hip_pose["rl_hip"][0], -leg_length["L1"] + hip_pose["rl_hip"][1], -150],
    "rr_foot": [hip_pose["rr_hip"][0], leg_length["L1"] + hip_pose["rr_hip"][1], -150],
}

left_pose = {
    "fl_foot": [hip_pose["fl_hip"][0]+100, -leg_length["L1"] + hip_pose["fl_hip"][1], -150],
    "fr_foot": [hip_pose["fr_hip"][0], leg_length["L1"] + hip_pose["fr_hip"][1], -150],
    "rl_foot": [hip_pose["rl_hip"][0], -leg_length["L1"] + hip_pose["rl_hip"][1], -150],
    "rr_foot": [hip_pose["rr_hip"][0], leg_length["L1"] + hip_pose["rr_hip"][1], -150],
}

right_pose = {
    "fl_foot": [hip_pose["fl_hip"][0], -leg_length["L1"] + hip_pose["fl_hip"][1], -150],
    "fr_foot": [hip_pose["fr_hip"][0]+100, leg_length["L1"] + hip_pose["fr_hip"][1], -150],
    "rl_foot": [hip_pose["rl_hip"][0], -leg_length["L1"] + hip_pose["rl_hip"][1], -150],
    "rr_foot": [hip_pose["rr_hip"][0], leg_length["L1"] + hip_pose["rr_hip"][1], -150],
}

