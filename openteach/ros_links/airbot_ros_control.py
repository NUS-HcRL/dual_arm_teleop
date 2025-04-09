import rospy
import numpy as np
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from copy import deepcopy as copy

class DexArmControl():
    def __init__(self):
        try:
            rospy.init_node("airbot_arm", disable_signals=True, anonymous=True)
        except Exception as e:
            rospy.logerr(f"Failed to initialize ROS node: {e}")

        self.robot_joint_state = None
        self.end_pose = None
        self.gripper_position = None

        # ROS subscribers and publishers
        rospy.Subscriber("/airbot_play/joint_states", JointState, self._callback_robot_joint_state)
        rospy.Subscriber("/airbot_play/end_pose", Pose, self._callback_end_pose)
        rospy.Subscriber("/airbot_play/gripper/position", Float64, self._callback_gripper_position)

        self.target_pose_pub = rospy.Publisher("/airbot_play/set_target_pose", Pose, queue_size=10)
        self.target_joint_pub = rospy.Publisher("/airbot_play/set_target_joint_q", JointState, queue_size=10)
        self.gripper_pub = rospy.Publisher("/airbot_play/gripper/set_position", Float64, queue_size=10)

    def _callback_robot_joint_state(self, joint_state):
        self.robot_joint_state = joint_state

    def _callback_end_pose(self, pose):
        self.end_pose = pose

    def _callback_gripper_position(self, position):
        self.gripper_position = position.data

    def get_robot_state(self):
        if self.robot_joint_state is None:
            return None

        raw_joint_state = copy(self.robot_joint_state)
        joint_state = dict(
            position=np.array(raw_joint_state.position, dtype=np.float32),
            velocity=np.array(raw_joint_state.velocity, dtype=np.float32),
            effort=np.array(raw_joint_state.effort, dtype=np.float32),
            timestamp=raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state

    def get_end_pose(self):
        if self.end_pose is None:
            return None

        pose_state = dict(
            position=np.array([self.end_pose.position.x,
                               self.end_pose.position.y,
                               self.end_pose.position.z], dtype=np.float32),
            orientation=np.array([self.end_pose.orientation.x,
                                  self.end_pose.orientation.y,
                                  self.end_pose.orientation.z,
                                  self.end_pose.orientation.w], dtype=np.float32)
        )
        return pose_state

    def get_gripper_position(self):
        return self.gripper_position

    def set_target_pose(self, position, orientation):
        pose_msg = Pose()
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = position
        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w = orientation
        self.target_pose_pub.publish(pose_msg)

    def set_target_joint_q(self, joint_positions):
        joint_state_msg = JointState()
        joint_state_msg.position = joint_positions
        self.target_joint_pub.publish(joint_state_msg)

    def set_gripper_position(self, position):
        position_msg = Float64()
        position_msg.data = position
        self.gripper_pub.publish(position_msg)
