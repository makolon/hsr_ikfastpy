import os
import re
import math
import random
import numpy as np
from itertools import combinations
from collections import namedtuple

from hsrb_never_collisions import NEVER_COLLISIONS
from sim_utils import multiply, get_link_pose, set_joint_position, set_joint_positions, get_joint_positions, get_min_limit, get_max_limit, quat_from_euler, read_pickle, set_pose, \
    get_pose, euler_from_quat, link_from_name, point_from_pose, invert, Pose, \
    unit_pose, joints_from_names, PoseSaver, get_aabb, get_joint_limits, ConfSaver, get_bodies, create_mesh, remove_body, \
    unit_from_theta, violates_limit, violates_limits, add_line, get_body_name, get_num_joints, approximate_as_cylinder, \
    approximate_as_prism, unit_quat, unit_point, angle_between, quat_from_pose, compute_jacobian, \
    movable_from_joints, quat_from_axis_angle, LockRenderer, Euler, get_links, get_link_name, \
    get_extend_fn, get_moving_links, link_pairs_collision, get_link_subtree, \
    clone_body, get_all_links, pairwise_collision, tform_point, get_camera_matrix, ray_from_pixel, pixel_from_ray, dimensions_from_camera_matrix, \
    wrap_angle, TRANSPARENT, PI, OOBB, pixel_from_point, set_all_color, wait_if_gui

ARM = 'arm'
ARM_NAMES = (ARM)

#####################################

HSR_GROUPS = {
    'base': ['joint_x', 'joint_y', 'joint_rz'],
    'torso': ['torso_lift_joint'],
    'head': ['head_pan_joint', 'head_tilt_joint'],
    'arm': ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'],
    'gripper': ['hand_l_spring_proximal_joint', 'hand_motor_joint', 'hand_r_spring_proximal_joint']
}
HSR_TOOL_FRAMES = {ARM: 'hand_palm_link'}
HSR_GRIPPER_ROOTS = {ARM: 'hand_palm_link'}
HSR_BASE_LINK = 'base_footprint'
HEAD_LINK_NAME = 'high_def_optical_frame'

# Arm tool poses
TOOL_POSE = Pose(euler=Euler(pitch=np.pi/2))

#####################################

# Special configurations

TOP_HOLDING_ARM = [0.1, -PI/4, 0.0, -PI/4, 0.0]
SIDE_HOLDING_ARM = [0.1, -PI/8, 0.0, -PI/8, 1.0]

HSR_CARRY_CONFS = {
    'top': TOP_HOLDING_ARM,
    'side': SIDE_HOLDING_ARM,
}

#####################################

HSRB_URDF = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'models/hsrb_description/robots/hsrb4s.urdf')

#####################################

def get_base_pose(hsr):
    return get_link_pose(hsr, link_from_name(hsr, HSR_BASE_LINK))

def arm_conf(arm, arm_config):
    if arm == ARM:
        return arm_config

def base_arm_conf(arm, base_config, arm_config):
    base_arm_conf = []
    if arm == ARM:
        for base in base_config:
            base_arm_conf.append(base)
        for arm in arm_config:
            base_arm_conf.append(arm)
        return base_arm_conf

def get_carry_conf(arm, grasp_type):
    return arm_conf(arm, HSR_CARRY_CONFS[grasp_type])

def get_other_arm(arm):
    for other_arm in ARM_NAMES:
        if other_arm != arm:
            return other_arm
    raise ValueError(arm)

#####################################

def get_disabled_collisions(hsr):
    disabled_names = NEVER_COLLISIONS
    link_mapping = {get_link_name(hsr, link): link for link in get_links(hsr)}
    return {(link_mapping[name1], link_mapping[name2])
            for name1, name2 in disabled_names if (name1 in link_mapping) and (name2 in link_mapping)}

def load_dae_collisions():
    dae_file = 'models/hsr_description/hsr-beta-static.dae'
    dae_string = open(dae_file).read()
    link_regex = r'<\s*link\s+sid="(\w+)"\s+name="(\w+)"\s*>'
    link_mapping = dict(re.findall(link_regex, dae_string))
    ignore_regex = r'<\s*ignore_link_pair\s+link0="kmodel1/(\w+)"\s+link1="kmodel1/(\w+)"\s*/>'
    disabled_collisions = []
    for link1, link2 in re.findall(ignore_regex, dae_string):
        disabled_collisions.append((link_mapping[link1], link_mapping[link2]))

    return disabled_collisions

def load_srdf_collisions():
    srdf_file = 'models/hsrb_description/hsrb4s.srdf'
    srdf_string = open(srdf_file).read()
    regex = r'<\s*disable_collisions\s+link1="(\w+)"\s+link2="(\w+)"\s+reason="(\w+)"\s*/>'
    disabled_collisions = []
    for link1, link2, reason in re.findall(regex, srdf_string):
        if reason == 'Never':
            disabled_collisions.append((link1, link2))

    return disabled_collisions

#####################################

def get_groups():
    return sorted(HSR_GROUPS)

def get_group_joints(robot, group):
    return joints_from_names(robot, HSR_GROUPS[group])

def get_group_conf(robot, group):
    return get_joint_positions(robot, get_group_joints(robot, group))

def set_group_conf(robot, group, positions):
    set_joint_positions(robot, get_group_joints(robot, group), positions)

def set_group_positions(robot, group_positions):
    for group, positions in group_positions.items():
        set_group_conf(robot, group, positions)

def get_group_positions(robot):
    return {group: get_group_conf(robot, group) for group in get_groups()}

#####################################

# End-effectors

def get_arm_joints(robot, arm):
    return get_group_joints(robot, arm)

def get_base_joints(robot, arm):
    return joints_from_names(robot, HSR_GROUPS['base'])

def get_torso_joints(robot, arm):
    return joints_from_names(robot, HSR_GROUPS['torso'])

def get_torso_arm_joints(robot, arm):
    return joints_from_names(robot, HSR_GROUPS['torso'] + HSR_GROUPS[arm])

def get_base_arm_joints(robot, arm):
    return joints_from_names(robot, HSR_GROUPS['base'] + HSR_GROUPS[arm])

def get_base_torso_joints(robot):
    return joints_from_names(robot, HSR_GROUPS['base'] + HSR_GROUPS['torso'])

def get_base_torso_arm_joints(robot):
    return joints_from_names(robot, HSR_GROUPS['base'] + HSR_GROUPS['torso'] + HSR_GROUPS[arm])

def set_arm_conf(robot, arm, conf):
    set_joint_positions(robot, get_arm_joints(robot, arm), conf)

def get_gripper_link(robot, arm):
    return link_from_name(robot, HSR_TOOL_FRAMES[arm])

def get_gripper_joints(robot, arm):
    return get_group_joints(robot, arm)

def set_gripper_position(robot, arm, position):
    gripper_joints = get_gripper_joints(robot, arm)
    set_joint_positions(robot, gripper_joints, [position] * len(gripper_joints))

def open_arm(robot, arm):
    for joint in get_gripper_joints(robot, arm):
        set_joint_position(robot, joint, get_max_limit(robot, joint))

def close_arm(robot, arm):
    for joint in get_gripper_joints(robot, arm):
        set_joint_position(robot, joint, get_min_limit(robot, joint))

open_gripper = open_arm
close_gripper = close_arm

#####################################

# Box grasps

GRASP_LENGTH = 0.
MAX_GRASP_WIDTH = np.inf
SIDE_HEIGHT_OFFSET = 0.03

def get_top_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                   max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH):
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)
    reflect_z = Pose(euler=[0, math.pi, 0])
    translate_z = Pose(point=[0, 0, h / 2 - grasp_length])
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    grasps = []

    if w <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=[0, 0, math.pi / 2 + i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]

    if l <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=[0, 0, i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]

    return grasps

def get_side_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                    max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH, top_offset=SIDE_HEIGHT_OFFSET):
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    grasps = []
    x_offset = h/2 - top_offset

    for j in range(1 + under):
        swap_xz = Pose(euler=[0, -math.pi / 2 + j * math.pi, 0])
        if w <= max_width:
            translate_z = Pose(point=[x_offset, 0, l / 2 - grasp_length])
            for i in range(2):
                rotate_z = Pose(euler=[math.pi / 2 + i * math.pi, 0, 0])
                grasps += [multiply(tool_pose, translate_z, rotate_z, swap_xz,
                                    translate_center, body_pose)]  # , np.array([w])

        if l <= max_width:
            translate_z = Pose(point=[x_offset, 0, w / 2 - grasp_length])
            for i in range(2):
                rotate_z = Pose(euler=[i * math.pi, 0, 0])
                grasps += [multiply(tool_pose, translate_z, rotate_z, swap_xz,
                                    translate_center, body_pose)]  # , np.array([l])

    return grasps

#####################################

# Cylinder grasps

def get_top_cylinder_grasps(body, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                            max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH):
    # Apply transformations right to left on object pose
    center, (diameter, height) = approximate_as_cylinder(body, body_pose=body_pose)
    reflect_z = Pose(euler=[0, math.pi, 0])
    translate_z = Pose(point=[0, 0, height / 2 - grasp_length])
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    if max_width < diameter:
        return

    while True:
        theta = random.uniform(0, 2*np.pi)
        rotate_z = Pose(euler=[0, 0, theta])
        yield multiply(tool_pose, translate_z, rotate_z,
                       reflect_z, translate_center, body_pose)

def get_side_cylinder_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                             max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH,
                             top_offset=SIDE_HEIGHT_OFFSET):
    center, (diameter, height) = approximate_as_cylinder(body, body_pose=body_pose)
    translate_center = Pose(point_from_pose(body_pose)-center)
    x_offset = height/2 - top_offset
    if max_width < diameter:
        return

    while True:
        theta = random.uniform(0, 2*np.pi)
        translate_rotate = ([x_offset, 0, diameter / 2 - grasp_length], quat_from_euler([theta, 0, 0]))
        for j in range(1 + under):
            swap_xz = Pose(euler=[0, -math.pi / 2 + j * math.pi, 0])
            yield multiply(tool_pose, translate_rotate, swap_xz, translate_center, body_pose)

def get_edge_cylinder_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                             grasp_length=GRASP_LENGTH):
    center, (diameter, height) = approximate_as_cylinder(body, body_pose=body_pose)
    translate_yz = Pose(point=[0, diameter/2, height/2 - grasp_length])
    reflect_y = Pose(euler=[0, math.pi, 0])
    translate_center = Pose(point=point_from_pose(body_pose)-center)

    while True:
        theta = random.uniform(0, 2*np.pi)
        rotate_z = Pose(euler=[0, 0, theta])
        for i in range(1 + under):
            rotate_under = Pose(euler=[0, 0, i * math.pi])
            yield multiply(tool_pose, rotate_under, translate_yz, rotate_z,
                           reflect_y, translate_center, body_pose)

#####################################

# Cylinder pushes

def get_cylinder_push(body, theta, under=False, body_quat=unit_quat(),
                      tilt=0., base_offset=0.02, side_offset=0.03):
    body_pose = (unit_point(), body_quat)
    center, (diameter, height) = approximate_as_cylinder(body, body_pose=body_pose)
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    tilt_gripper = Pose(euler=Euler(pitch=tilt))
    translate_x = Pose(point=[-diameter / 2 - side_offset, 0, 0]) # Compute as a function of theta
    translate_z = Pose(point=[0, 0, -height / 2 + base_offset])
    rotate_x = Pose(euler=Euler(yaw=theta))
    reflect_z = Pose(euler=Euler(pitch=math.pi))
    grasps = []
    for i in range(1 + under):
        rotate_z = Pose(euler=Euler(yaw=i * math.pi))
        grasps.append(multiply(tilt_gripper, translate_x, translate_z, rotate_x, rotate_z,
                               reflect_z, translate_center, body_pose))

    return grasps

#####################################

# Button presses

PRESS_OFFSET = 0.02

def get_x_presses(body, max_orientations=1, body_pose=unit_pose(), top_offset=PRESS_OFFSET):
    # gripper_from_object
    center, (w, _, h) = approximate_as_prism(body, body_pose=body_pose)
    translate_center = Pose(-center)
    press_poses = []
    for j in range(max_orientations):
        swap_xz = Pose(euler=[0, -math.pi / 2 + j * math.pi, 0])
        translate = Pose(point=[0, 0, w / 2 + top_offset])
        press_poses += [multiply(TOOL_POSE, translate, swap_xz, translate_center, body_pose)]

    return press_poses

def get_top_presses(body, tool_pose=TOOL_POSE, body_pose=unit_pose(), top_offset=PRESS_OFFSET, **kwargs):
    center, (_, height) = approximate_as_cylinder(body, body_pose=body_pose, **kwargs)
    reflect_z = Pose(euler=[0, math.pi, 0])
    translate_z = Pose(point=[0, 0, height / 2 + top_offset])
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    while True:
        theta = random.uniform(0, 2*np.pi)
        rotate_z = Pose(euler=[0, 0, theta])
        yield multiply(tool_pose, translate_z, rotate_z,
                       reflect_z, translate_center, body_pose)

GET_GRASPS = {
    'top': get_top_grasps,
    'side': get_side_grasps,
}

#####################################

# Inverse reachability

DATABASES_DIR = '../databases'
IR_FILENAME = '{}_{}_ir.pickle'
IR_CACHE = {}

def get_database_file(filename):
    directory = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(directory, DATABASES_DIR, filename)


def load_inverse_reachability(arm, grasp_type):
    key =  (arm, grasp_type)
    if key not in IR_CACHE:
        filename = IR_FILENAME.format(grasp_type, arm)
        path = get_database_file(filename)
        IR_CACHE[key] = read_pickle(path)['gripper_from_base']

    return IR_CACHE[key]


def learned_forward_generator(robot, base_pose, arm, grasp_type):
    gripper_from_base_list = list(load_inverse_reachability(arm, grasp_type))
    random.shuffle(gripper_from_base_list)
    for gripper_from_base in gripper_from_base_list:
        yield multiply(base_pose, invert(gripper_from_base))


def learned_pose_generator(robot, gripper_pose, arm, grasp_type):
    gripper_from_base_list = load_inverse_reachability(arm, grasp_type)
    random.shuffle(gripper_from_base_list)
    for gripper_from_base in gripper_from_base_list:
        base_point, base_quat = multiply(gripper_pose, gripper_from_base)
        x, y, _ = base_point
        _, _, theta = euler_from_quat(base_quat)
        base_values = (x, y, theta)
        yield base_values

#####################################

# Camera

MAX_VISUAL_DISTANCE = 5.0
MAX_KINECT_DISTANCE = 2.5

HSR_CAMERA_MATRIX = get_camera_matrix(
    width=640, height=480, fx=772.55, fy=772.5)

def get_hsr_view_section(z, camera_matrix=None):
    if camera_matrix is None:
        camera_matrix = HSR_CAMERA_MATRIX
    width, height = dimensions_from_camera_matrix(camera_matrix)
    pixels = [(0, 0), (width, height)]
    return [z*ray_from_pixel(camera_matrix, p) for p in pixels]

def get_hsr_field_of_view(**kwargs):
    z = 1
    view_lower, view_upper = get_hsr_view_section(z=z, **kwargs)
    horizontal = angle_between([view_lower[0], 0, z],
                               [view_upper[0], 0, z])
    vertical = angle_between([0, view_lower[1], z],
                             [0, view_upper[1], z])
    return horizontal, vertical

def is_visible_point(camera_matrix, depth, point_world, camera_pose=unit_pose()):
    point_camera = tform_point(invert(camera_pose), point_world)
    if not (0 <= point_camera[2] < depth):
        return False
    pixel = pixel_from_point(camera_matrix, point_camera)
    return pixel is not None

def is_visible_aabb(aabb, **kwargs):
    body_lower, body_upper = aabb
    z = body_lower[2]
    if z < 0:
        return False
    view_lower, view_upper = get_hsr_view_section(z, **kwargs)
    return not (np.any(body_lower[:2] < view_lower[:2]) or
                np.any(view_upper[:2] < body_upper[:2]))

def support_from_aabb(aabb, near=True):
    lower, upper = aabb
    min_x, min_y, min_z = lower
    max_x, max_y, max_z = upper
    z = min_z if near else max_z
    return [(min_x, min_y, z), (min_x, max_y, z),
            (max_x, max_y, z), (max_x, min_y, z)]

#####################################

def cone_vertices_from_base(base):
    return [np.zeros(3)] + base

def cone_wires_from_support(support, cone_only=True):
    apex = np.zeros(3)
    lines = []
    for vertex in support:
        lines.append((apex, vertex))
    if cone_only:
        for i, v2 in enumerate(support):
           v1 = support[i-1]
           lines.append((v1, v2))
    else:
        for v1, v2 in combinations(support, 2):
            lines.append((v1, v2))
        center = np.average(support, axis=0)
        lines.append((apex, center))

    return lines

def cone_mesh_from_support(support):
    assert(len(support) == 4)
    vertices = cone_vertices_from_base(support)
    faces = [(1, 4, 3), (1, 3, 2)]
    for i in range(len(support)):
        index1 = 1+i
        index2 = 1+(i+1)%len(support)
        faces.append((0, index1, index2))

    return vertices, faces

def get_viewcone_base(depth=MAX_VISUAL_DISTANCE, camera_matrix=None):
    if camera_matrix is None:
        camera_matrix = HSR_CAMERA_MATRIX
    width, height = dimensions_from_camera_matrix(camera_matrix)
    vertices = []
    for pixel in [(0, 0), (width, 0), (width, height), (0, height)]:
        ray = depth * ray_from_pixel(camera_matrix, pixel)
        vertices.append(ray[:3])

    return vertices

def get_viewcone(depth=MAX_VISUAL_DISTANCE, camera_matrix=None, **kwargs):
    mesh = cone_mesh_from_support(get_viewcone_base(
        depth=depth, camera_matrix=camera_matrix))
    assert (mesh is not None)

    return create_mesh(mesh, **kwargs)

def attach_viewcone(robot, head_name=HEAD_LINK_NAME, depth=MAX_VISUAL_DISTANCE,
                    camera_matrix=None, color=(1, 0, 0), **kwargs):
    head_link = link_from_name(robot, head_name)
    lines = []
    for v1, v2 in cone_wires_from_support(get_viewcone_base(
            depth=depth, camera_matrix=camera_matrix)):
        if is_optical(head_name):
            rotation = Pose()
        else:
            rotation = Pose(euler=Euler(roll=-np.pi/2, yaw=-np.pi/2)) # Apply in reverse order
        p1 = tform_point(rotation, v1)
        p2 = tform_point(rotation, v2)
        lines.append(add_line(p1, p2, color=color, parent=robot, parent_link=head_link, **kwargs))

    return lines

def draw_viewcone(pose, depth=MAX_VISUAL_DISTANCE,
                  camera_matrix=None, color=(1, 0, 0), **kwargs):
    lines = []
    for v1, v2 in cone_wires_from_support(get_viewcone_base(
            depth=depth, camera_matrix=camera_matrix)):
        p1 = tform_point(pose, v1)
        p2 = tform_point(pose, v2)
        lines.append(add_line(p1, p2, color=color, **kwargs))

    return lines

#####################################

def is_optical(link_name):
    return 'optical' in link_name

def inverse_visibility(hsr, point, head_name=HEAD_LINK_NAME, head_joints=None,
                       max_iterations=100, step_size=0.5, tolerance=np.pi*1e-2, verbose=False):
    head_link = link_from_name(hsr, head_name)
    camera_axis = np.array([0, 0, 1]) if is_optical(head_name) else np.array([1, 0, 0])
    if head_joints is None:
        head_joints = joints_from_names(hsr, HSR_GROUPS['head'])
    head_conf = np.zeros(len(head_joints))
    with LockRenderer(lock=True):
        with ConfSaver(hsr):
            for iteration in range(max_iterations):
                set_joint_positions(hsr, head_joints, head_conf)
                world_from_head = get_link_pose(hsr, head_link)
                point_head = tform_point(invert(world_from_head), point)
                error_angle = angle_between(camera_axis, point_head)
                if abs(error_angle) <= tolerance:
                    break
                normal_head = np.cross(camera_axis, point_head)
                normal_world = tform_point((unit_point(), quat_from_pose(world_from_head)), normal_head)
                correction_quat = quat_from_axis_angle(normal_world, step_size*error_angle)
                correction_euler = euler_from_quat(correction_quat)
                _, angular = compute_jacobian(hsr, head_link)
                correction_conf = np.array([np.dot(angular[mj], correction_euler)
                                            for mj in movable_from_joints(hsr, head_joints)])
                if verbose:
                    print('Iteration: {} | Error: {:.3f} | Correction: {}'.format(
                        iteration, error_angle, correction_conf))
                head_conf += correction_conf

                if np.all(correction_conf == 0):
                    return None
            else:
                return None
    if violates_limits(hsr, head_joints, head_conf):
        return None
    return head_conf

def plan_scan_path(hsr, tilt=0):
    head_joints = joints_from_names(hsr, HSR_GROUPS['head'])
    start_conf = get_joint_positions(hsr, head_joints)
    lower_limit, upper_limit = get_joint_limits(hsr, head_joints[0])

    first_conf = np.array([lower_limit, tilt])
    second_conf = np.array([upper_limit, tilt])
    if start_conf[0] > 0:
        first_conf, second_conf = second_conf, first_conf
    return [first_conf, second_conf]

def plan_pause_scan_path(hsr, tilt=0):
    head_joints = joints_from_names(hsr, HSR_GROUPS['head'])
    assert(not violates_limit(hsr, head_joints[1], tilt))
    theta, _ = get_hsr_field_of_view()
    lower_limit, upper_limit = get_joint_limits(hsr, head_joints[0])
    # Add one because half visible on limits
    n = int(np.math.ceil((upper_limit - lower_limit) / theta) + 1)
    epsilon = 1e-3
    return [np.array([pan, tilt]) for pan in np.linspace(lower_limit + epsilon,
                                                         upper_limit - epsilon, n, endpoint=True)]

#####################################

Detection = namedtuple('Detection', ['body', 'distance'])

def get_view_aabb(body, view_pose, **kwargs):
    with PoseSaver(body):
        body_view = multiply(invert(view_pose), get_pose(body))
        set_pose(body, body_view)
        return get_aabb(body, **kwargs)

def get_view_oobb(body, view_pose, **kwargs):
    return OOBB(get_view_aabb(body, view_pose, **kwargs), view_pose)

def get_detection_cone(hsr, body, camera_link=HEAD_LINK_NAME, depth=MAX_VISUAL_DISTANCE, **kwargs):
    head_link = link_from_name(hsr, camera_link)
    body_aabb = get_view_aabb(body, get_link_pose(hsr, head_link))
    lower_z = body_aabb[0][2]
    if depth < lower_z:
        return None, lower_z
    if not is_visible_aabb(body_aabb, **kwargs):
        return None, lower_z
    return cone_mesh_from_support(support_from_aabb(body_aabb)), lower_z

def get_detections(hsr, p_false_neg=0, camera_link=HEAD_LINK_NAME,
                   exclude_links=set(), color=None, **kwargs):
    camera_pose = get_link_pose(hsr, link_from_name(hsr, camera_link))
    detections = []
    for body in get_bodies():
        if (hsr == body) or (np.random.random() < p_false_neg):
            continue
        mesh, z = get_detection_cone(hsr, body, camera_link=camera_link, **kwargs)
        if mesh is None:
            continue
        cone = create_mesh(mesh, color=color)
        set_pose(cone, camera_pose)
        if not any(pairwise_collision(cone, obst)
                   for obst in set(get_bodies()) - {hsr, body, cone}) \
                and not any(link_pairs_collision(hsr, [link], cone)
                            for link in set(get_all_links(hsr)) - exclude_links):
            detections.append(Detection(body, z))
        remove_body(cone)
    return detections

def get_visual_detections(hsr, **kwargs):
    return [body for body, _ in get_detections(hsr, depth=MAX_VISUAL_DISTANCE, **kwargs)]

def get_kinect_registrations(hsr, **kwargs):
    return [body for body, _ in get_detections(hsr, depth=MAX_KINECT_DISTANCE, **kwargs)]

#####################################

def visible_base_generator(robot, target_point, base_range=(1., 1.), theta_range=(0., 0.)):
    while True:
        base_from_target = unit_from_theta(np.random.uniform(0., 2 * np.pi))
        look_distance = np.random.uniform(*base_range)
        base_xy = target_point[:2] - look_distance * base_from_target
        base_theta = np.math.atan2(base_from_target[1], base_from_target[0]) + np.random.uniform(*theta_range)
        base_q = np.append(base_xy, wrap_angle(base_theta))
        yield base_q

def get_base_extend_fn(robot):
    raise NotImplementedError()

#####################################

def close_until_collision(robot, gripper_joints, bodies=[], open_conf=None, closed_conf=None, num_steps=25, **kwargs):
    if not gripper_joints:
        return None
    if open_conf is None:
        open_conf = [get_max_limit(robot, joint) for joint in gripper_joints]
    if closed_conf is None:
        closed_conf = [get_min_limit(robot, joint) for joint in gripper_joints]
    resolutions = np.abs(np.array(open_conf) - np.array(closed_conf)) / num_steps
    extend_fn = get_extend_fn(robot, gripper_joints, resolutions=resolutions)
    close_path = [open_conf] + list(extend_fn(open_conf, closed_conf))
    collision_links = frozenset(get_moving_links(robot, gripper_joints))

    for i, conf in enumerate(close_path):
        set_joint_positions(robot, gripper_joints, conf)
        if any(pairwise_collision((robot, collision_links), body, **kwargs) for body in bodies):
            if i == 0:
                return None
            return close_path[i-1][0]
    return close_path[-1][0]

def compute_grasp_width(robot, arm, body, grasp_pose, **kwargs):
    tool_link = get_gripper_link(robot, arm)
    tool_pose = get_link_pose(robot, tool_link)
    body_pose = multiply(tool_pose, grasp_pose)
    set_pose(body, body_pose)
    gripper_joints = get_gripper_joints(robot, arm)
    return close_until_collision(robot, gripper_joints, bodies=[body], **kwargs)

def create_gripper(robot, arm, visual=True):
    link_name = HSR_GRIPPER_ROOTS[arm]
    links = get_link_subtree(robot, link_from_name(robot, link_name))
    gripper = clone_body(robot, links=links, visual=False, collision=True)
    if not visual:
        set_all_color(robot, TRANSPARENT)
    return gripper
