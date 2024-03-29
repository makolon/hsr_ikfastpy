import rospy
import numpy as np
import hsrb_interface
from scipy.spatial.transform import Rotation as R

robot = hsrb_interface.Robot()
base = robot.get('omni_base')
gripper = robot.get('gripper')
whole_body = robot.get('whole_body')

def get_link_pose(link):
    tf_pose = whole_body._tf2_buffer.lookup_transform('map', link, rospy.Time(0))
    link_pose = ((tf_pose.transform.translation.x,
                  tf_pose.transform.translation.y,
                  tf_pose.transform.translation.z),
                 (tf_pose.transform.rotation.x,
                  tf_pose.transform.rotation.y,
                  tf_pose.transform.rotation.z,
                  tf_pose.transform.rotation.w))
    return link_pose

def get_joint_limits(joint):
    if joint == 'odom_x':
        limit = (-10.0, 10.0)
    elif joint == 'odom_y':
        limit = (-10.0, 10.0)
    elif joint == 'odom_t':
        limit = (-10.0, 10.0)
    else:
        limit = whole_body.joint_limits[joint]
    return limit

def get_custom_limits(base_joints, arm_joints, custom_limits={}):
    joint_limits = []
    for joint in base_joints:
        if joint in custom_limits:
            joint_limits.append(custom_limits[joint])
        else:
            joint_limits.append(get_joint_limits(joint))
    for joint in arm_joints:
        if joint in custom_limits:
            joint_limits.append(custom_limits[joint])
        else:
            joint_limits.append(get_joint_limits(joint))
    return zip(*joint_limits)

def get_distance(p1, p2, **kwargs):
    assert len(p1) == len(p2)
    diff = np.array(p2) - np.array(p1)
    return np.linalg.norm(diff, ord=2)

def get_joint_position(joint):
    if joint == 'world_joint':
        joint_position = base.pose
    else:
        joint_position = whole_body.joint_positions[joint]
    return joint_position

def get_joint_positions(jonits):
    joint_positions = []
    for joint in jonits:
        if joint == 'world_joint':
            base_pose = base._tf2_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            joint_positions.append(base_pose.transform.translation.x)
            joint_positions.append(base_pose.transform.translation.y)
            base_quat = np.array([base_pose.transform.rotation.x,
                   base_pose.transform.rotation.y,
                   base_pose.transform.rotation.z,
                   base_pose.transform.rotation.w])
            base_rz = R.from_quat(base_quat).as_euler('xyz')[2]
            joint_positions.append(base_rz)
        else:
            joint_positions.append(whole_body.joint_positions[joint])
    return joint_positions