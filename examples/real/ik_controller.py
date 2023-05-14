import rospy
import numpy as np
from ik_solver import get_tool_pose, get_ik_generator, hsr_inverse_kinematics
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class IKController():
    def __init__(self):
        # Publisher
        self.arm_pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.base_pub = rospy.Publisher('/hsrb/omni_base_controller/command', JointTrajectory, queue_size=10)

        # Wait for publisher has built
        while self.base_pub.get_num_connections() == 0:
            rospy.sleep(0.1)
        while self.arm_pub.get_num_connections() == 0:
            rospy.sleep(0.1)

    def set_pose(self, base_pose, joint_pose):
        base_traj = JointTrajectory()
        arm_traj = JointTrajectory()

        base_traj.joint_names = ['odom_x', 'odom_y', 'odom_t']
        arm_traj.joint_names = ['arm_lift_joint', 'arm_flex_joint',
                                'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']

        # Set base trajectory
        assert len(base_pose) == 3, "Does not match the size of base pose"
        base_p = JointTrajectoryPoint()
        base_p.positions = base_pose
        base_p.velocities = np.zeros(len(base_pose))
        base_p.time_from_start = rospy.Duration(1)
        base_traj.points = [base_p]

        # Set arm trajectory
        assert len(joint_pose) == 5, "Does not match the size of base pose"
        arm_p = JointTrajectoryPoint()
        arm_p.positions = joint_pose
        arm_p.velocities = np.zeros(len(joint_pose))
        arm_p.time_from_start = rospy.Duration(1)
        arm_traj.points = [arm_p]

        return base_traj, arm_traj

    def control(self, pose):
        # Inverse kinematics
        generator = get_ik_generator('arm', pose)
        ik_pose = next(generator)

        if len(ik_pose) == 0:
            return
        else:
            base_pose, arm_pose = ik_pose[0][:3], ik_pose[0][3:]

        # Set target pose
        base_traj, arm_traj = self.set_pose(base_pose, arm_pose)

        # Publish target pose
        self.base_pub.publish(base_traj)
        self.arm_pub.publish(arm_traj)