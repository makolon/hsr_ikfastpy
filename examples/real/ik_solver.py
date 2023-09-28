import os
import sys
import glob
import random
import numpy as np
from scipy.spatial.transform import Rotation as R

from .utils import multiply, invert, all_between, compute_forward_kinematics, \
    compute_inverse_kinematics, select_solution, USE_ALL 
from .hsrb_utils import get_link_pose, get_joint_positions, get_custom_limits


BASE_FRAME = 'base_footprint'
TORSO_JOINT = 'torso_lift_joint'
ROTATION_JOINT = 'joint_rz'
LIFT_JOINT = 'arm_lift_joint'
HSR_TOOL_FRAMES = {'arm': 'hand_palm_link'}
IK_FRAME = {'arm': 'hand_palm_link'}

def get_ik_lib():
    lib_path = os.environ['PYTHONPATH'].split(':')[1] # TODO: modify
    ik_lib_path = glob.glob(lib_path, recursive=True)
    return ik_lib_path[0]

#####################################

def get_tool_pose(arm):
    sys.path.append(get_ik_lib())
    from ikArm import armFK

    arm_fk = {'arm': armFK}
    ik_joints = ['world_joint', 'arm_lift_joint', 'arm_flex_joint', 
                 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
    conf = get_joint_positions(ik_joints)
    assert len(conf) == 8

    base_from_tool = compute_forward_kinematics(arm_fk[arm], conf)
    world_from_base = get_link_pose(BASE_FRAME)
    return multiply(world_from_base, base_from_tool)

#####################################

def get_ik_generator(arm, ik_pose, custom_limits={}):
    sys.path.append(get_ik_lib())
    from ikArm import armIK

    arm_ik = {'arm': armIK}

    base_joints = ['odom_x', 'odom_y', 'odom_t']
    arm_joints = ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
    min_limits, max_limits = get_custom_limits(base_joints, arm_joints, custom_limits)

    arm_rot = R.from_quat(ik_pose[1]).as_euler('xyz')[0]
    sampled_limits = [(arm_rot-np.pi, arm_rot-np.pi), (0.0, 0.34)]
    while True:
        sampled_values = [random.uniform(*limits) for limits in sampled_limits]
        confs = compute_inverse_kinematics(arm_ik[arm], ik_pose, sampled_values)
        solutions = [q for q in confs if all_between(min_limits, q, max_limits)]
        yield solutions
        if all(lower == upper for lower, upper in sampled_limits):
            break

def get_tool_from_ik(arm):
    world_from_tool = get_link_pose(HSR_TOOL_FRAMES[arm])
    world_from_ik = get_link_pose(IK_FRAME[arm])
    return multiply(invert(world_from_tool), world_from_ik)

def sample_tool_ik(arm, tool_pose, nearby_conf=USE_ALL, max_attempts=100, **kwargs):
    generator = get_ik_generator(arm, tool_pose, **kwargs)
    whole_body_joints = ['world_joint', 'torso_lift_joint', 'arm_lift_joint', 'arm_flex_joint', 
                         'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']

    for _ in range(max_attempts):
        try:
            solutions = next(generator)
            if solutions:
                return select_solution(whole_body_joints, solutions, nearby_conf=nearby_conf)
        except StopIteration:
            break

    return None

def hsr_inverse_kinematics(arm, gripper_pose, custom_limits={}, **kwargs):
    base_arm_conf = sample_tool_ik(arm, gripper_pose, custom_limits=custom_limits, **kwargs)
    if base_arm_conf is None:
        return None

    return base_arm_conf