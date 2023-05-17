#!/usr/bin/env python

import os
import numpy as np
import pybullet as p

from hsrb_utils import TOP_HOLDING_ARM, SIDE_HOLDING_ARM, HSRB_URDF, \
    HSR_GROUPS, get_disabled_collisions
from sim_utils import set_base_values, quat_from_euler, \
    set_joint_positions, add_data_path, connect, plan_base_motion, plan_joint_motion, enable_gravity, \
    joint_controller, dump_body, load_model, joints_from_names, wait_if_gui, disconnect, \
    get_link_pose, link_from_name, wait_if_gui, load_pybullet, set_quat, Euler, PI, RED, add_line, \
    wait_for_duration, LockRenderer, HideOutput

SLEEP = None

def test_base_motion(hsr, base_start, base_goal, obstacles=[]):
    set_base_values(hsr, base_start)
    wait_if_gui('Plan Base?')
    base_limits = ((-2.5, -2.5), (2.5, 2.5))
    with LockRenderer(lock=False):
        base_path = plan_base_motion(hsr, base_goal, base_limits, obstacles=obstacles)
    if base_path is None:
        print('Unable to find a base path')
        return
    print(len(base_path))
    for bq in base_path:
        set_base_values(hsr, bq)
        if SLEEP is None:
            wait_if_gui('Continue?')
        else:
            wait_for_duration(SLEEP)

#####################################

def test_arm_motion(hsr, arm_joints, arm_goal):
    disabled_collisions = get_disabled_collisions(hsr)
    wait_if_gui('Plan Arm?')
    with LockRenderer(lock=False):
        arm_path = plan_joint_motion(hsr, arm_joints, arm_goal, disabled_collisions=disabled_collisions)
    if arm_path is None:
        print('Unable to find an arm path')
        return
    print(len(arm_path))
    for q in arm_path:
        set_joint_positions(hsr, arm_joints, q)
        wait_for_duration(0.01)

def test_arm_control(hsr, arm_joints, arm_start):
    wait_if_gui('Control Arm?')
    real_time = False
    enable_gravity()
    p.setRealTimeSimulation(real_time)
    for _ in joint_controller(hsr, arm_joints, arm_start):
        if not real_time:
            p.stepSimulation()

#####################################

def test_ikfast(hsr):
    from ik_solver import get_tool_pose, get_ikfast_generator
    arm_joints = joints_from_names(hsr, HSR_GROUPS['arm'])
    base_joints = joints_from_names(hsr, HSR_GROUPS['base'])
    base_arm = base_joints + arm_joints

    arm = 'arm'
    pose = get_tool_pose(hsr, arm)

    print('get_link_pose: ', get_link_pose(hsr, link_from_name(hsr, 'hand_palm_link')))
    print('get_tool_pose: ', pose)
    for i in range(1000):
        pose_x = 2.5 + np.random.random() * 0.1
        pose_y = 2.0 + np.random.random() * 0.1
        pose_z = 0.6 + np.random.random() * 0.1
        tool_pose = ((pose_x, pose_y, pose_z), (0.707107, 0.0, 0.707107, 0.0))
        generator = get_ikfast_generator(hsr, arm, tool_pose, torso_limits=False)
        solutions = next(generator)
        print(i, len(solutions))
        for q in solutions:
            set_joint_positions(hsr, base_arm, q)
            wait_if_gui()

#####################################

def main():
    connect(use_gui=True)
    add_data_path()

    plane = p.loadURDF("plane.urdf")

    directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    add_data_path(directory)

    table_path = "models/table_collision/table.urdf"
    table = load_pybullet(table_path, fixed_base=True)
    set_quat(table, quat_from_euler(Euler(yaw=PI/2)))
    obstacles = [table]

    hsr_urdf = HSRB_URDF
    with HideOutput():
        hsr = load_model(hsr_urdf, fixed_base=True)
    dump_body(hsr)

    base_start = (-2, -2, 0)
    base_goal = (2, 2, 0)
    arm_start = SIDE_HOLDING_ARM
    arm_goal = TOP_HOLDING_ARM

    arm_joints = joints_from_names(hsr, HSR_GROUPS['arm'])
    torso_joints = joints_from_names(hsr, HSR_GROUPS['torso'])
    gripper_joints = joints_from_names(hsr, HSR_GROUPS['gripper'])

    print('Set joints')
    set_joint_positions(hsr, arm_joints, arm_start)
    set_joint_positions(hsr, torso_joints, [0.0])
    set_joint_positions(hsr, gripper_joints, [0, 1, 0])

    add_line(base_start, base_goal, color=RED)
    print(base_start, base_goal)

    print('Test base motion')
    test_base_motion(hsr, base_start, base_goal, obstacles=obstacles)

    print('Test arm motion')
    test_arm_motion(hsr, arm_joints, arm_goal)

    test_ikfast(hsr)

    while True:
        word = input('Input something: ')
        if word == 'Finish':
            disconnect()
            break

if __name__ == '__main__':
    main()
