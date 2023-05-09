import random

from hsrb_utils import get_torso_arm_joints, get_base_joints, get_gripper_link, get_arm_joints, \
    get_base_arm_joints, get_link_pose, get_joint_positions, get_custom_limits_with_base, \
    joint_from_name, link_from_name 
from utils import multiply, invert, all_between, compute_forward_kinematics, compute_inverse_kinematics, \
    get_ik_limits, select_solution, USE_ALL, USE_CURRENT

BASE_FRAME = 'base_link'
TORSO_JOINT = 'torso_lift_joint'
HSR_TOOL_FRAMES = {'arm': 'hand_palm_link'}
IK_FRAME = {'arm': 'hand_palm_link'}

#####################################

def get_tool_pose(robot, arm):
    from .ikArm import armFK
    arm_fk = {'arm': armFK}
    ik_joints = get_base_arm_joints(robot, arm)
    conf = get_joint_positions(robot, ik_joints)
    assert len(conf) == 8

    base_from_tool = compute_forward_kinematics(arm_fk[arm], conf)
    world_from_base = get_link_pose(robot, link_from_name(robot, BASE_FRAME))
    return multiply(world_from_base, base_from_tool)

#####################################

def get_ik_generator(robot, arm, ik_pose, torso_limits=USE_ALL, upper_limits=USE_ALL, custom_limits={}):
    from .ikArm import armIK

    arm_ik = {'arm': armIK}
    world_from_base = get_link_pose(robot, link_from_name(robot, BASE_FRAME))
    base_from_ik = multiply(invert(world_from_base), ik_pose)
    sampled_joints = [joint_from_name(robot, name) for name in [TORSO_JOINT]]
    sampled_limits = [get_ik_limits(robot, joint, limits) for joint, limits in zip(sampled_joints, [torso_limits])]
    arm_joints = get_arm_joints(robot, arm)
    base_joints = get_base_joints(robot, arm)
    min_limits, max_limits = get_custom_limits_with_base(robot, arm_joints, base_joints, custom_limits)

    while True:
        sampled_values = [random.uniform(*limits) for limits in sampled_limits]
        confs = compute_inverse_kinematics(arm_ik[arm], base_from_ik, sampled_values)
        solutions = [q for q in confs if all_between(min_limits, q, max_limits)]
        yield solutions
        if all(lower == upper for lower, upper in sampled_limits):
            break

def get_tool_from_ik(robot, arm):
    world_from_tool = get_link_pose(robot, link_from_name(robot, HSR_TOOL_FRAMES[arm]))
    world_from_ik = get_link_pose(robot, link_from_name(robot, IK_FRAME[arm]))
    return multiply(invert(world_from_tool), world_from_ik)

def sample_tool_ik(robot, arm, tool_pose, nearby_conf=USE_ALL, max_attempts=100, **kwargs):
    ik_pose = multiply(tool_pose, get_tool_from_ik(robot, arm))

    generator = get_ik_generator(robot, arm, ik_pose, **kwargs)
    arm_joints = get_torso_arm_joints(robot, arm)

    for _ in range(max_attempts):
        try:
            solutions = next(generator)
            if solutions:
                return select_solution(robot, arm_joints, solutions, nearby_conf=nearby_conf)
        except StopIteration:
            break

    return None

def hsr_inverse_kinematics(robot, arm, gripper_pose, custom_limits={}, **kwargs):
    arm_link = get_gripper_link(robot, arm)
    arm_joints = get_arm_joints(robot, arm)
    base_arm_joints = get_base_arm_joints(robot, arm)

    base_arm_conf = sample_tool_ik(robot,
                                    arm,
                                    gripper_pose,
                                    custom_limits=custom_limits,
                                    torso_limits=USE_CURRENT,
                                    **kwargs)
    if base_arm_conf is None:
        return None

    return get_joint_positions(robot, base_arm_joints)