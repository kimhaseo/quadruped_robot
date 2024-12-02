
leg_length = {
    "L1": 89,
    "L2": 170,
    "L3": 200
}

body_dimensions = {
    "length": 430,  # mm
    "width": 110,   # mm
    "height": 250   # mm
}

leg_resolution = 200

init_pose = {
    "fl_coord": [0.0, -leg_length["L1"], -body_dimensions["height"]],  # 관절 각도
    "fr_coord": [0.0, leg_length["L1"], -body_dimensions["height"]],
    "rl_coord": [0.0, -leg_length["L1"], -body_dimensions["height"]],
    "rr_coord": [0.0, leg_length["L1"], -body_dimensions["height"]],
}

start_pose = {
    "fl_coord": [0.0, -leg_length["L1"], -70],  # 관절 각도
    "fr_coord": [0.0, leg_length["L1"], -70],
    "rl_coord": [0.0, -leg_length["L1"], -70],
    "rr_coord": [0.0, leg_length["L1"], -70],
}

current_pose = {
    "fl_coord": [0.0, -leg_length["L1"], -70],  # 관절 각도
    "fr_coord": [0.0, leg_length["L1"], -70],
    "rl_coord": [0.0, -leg_length["L1"], -70],
    "rr_coord": [0.0, leg_length["L1"], -70],
}
