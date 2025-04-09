import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64

class AirbotArm:
    def __init__(self, record_type=None):
        """初始化 AirbotArm控制器"""
        rospy.init_node("airbot_arm_control", anonymous=True, disable_signals=True)
        self._data_frequency = 60
        self.joint_state = None
        self.cartesian_pose = None

        # Subscribers and publishers
        self.joint_state_sub = rospy.Subscriber("/airbot_play/joint_states", JointState, self._callback_joint_state)
        self.cartesian_pose_sub = rospy.Subscriber("/airbot_play/end_pose", Pose, self._callback_cartesian_pose)
        self.joint_command_pub = rospy.Publisher("/airbot_play/set_target_joint_q", JointState, queue_size=10)
        self.pose_command_pub = rospy.Publisher("/airbot_play/set_target_pose", Pose, queue_size=10)

    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_joint_state,
            'cartesian_states': self.get_cartesian_state
        }

    @property
    def name(self):
        return 'airbot'

    @property
    def data_frequency(self):
        return self._data_frequency

    # Callback functions
    def _callback_joint_state(self, msg):
        self.joint_state = msg

    def _callback_cartesian_pose(self, msg):
        self.cartesian_pose = msg


    def get_joint_state(self):
        # 同步获取关节状态
        try:
            joint_state_msg = rospy.wait_for_message("/airbot_play/joint_states", JointState, timeout=1.0)
            return {
                'position': np.array(joint_state_msg.position),
                'velocity': np.array(joint_state_msg.velocity),
                'effort': np.array(joint_state_msg.effort),
            }
        except rospy.ROSException:
            rospy.logwarn("Failed to receive joint state message.")
            return None

    def get_cartesian_state(self):
        try:
            cartesian_pose_msg = rospy.wait_for_message("/airbot_play/end_pose", Pose, timeout=1.0)
            return (
                np.array([cartesian_pose_msg.position.x, cartesian_pose_msg.position.y, cartesian_pose_msg.position.z]),
                np.array([cartesian_pose_msg.orientation.x, cartesian_pose_msg.orientation.y, cartesian_pose_msg.orientation.z, cartesian_pose_msg.orientation.w]),
            )
        except rospy.ROSException:
            rospy.logwarn("Failed to receive cartesian pose message.")
            return None

        
    def get_joint_position(self):
        state = self.get_joint_state()
        return state['position'] if state else None

    def get_joint_velocity(self):
        state = self.get_joint_state()
        return state['velocity'] if state else None

    def get_joint_torque(self):
        state = self.get_joint_state()
        return state['effort'] if state else None

    def get_cartesian_position(self):
        state = self.get_cartesian_state()
        return state['position'] if state else None

    # def move_gripper(self, open):
    #     target_end = 1.0 if open else 0.0
    #     # TODO: Publish gripper command via a specific ROS topic if available.
    #     # rospy.loginfo(f"Gripper command not implemented. Target: {target_end}")
    #     return True
    def move_gripper(self, open):

        # 目标位置: 1.0 表示打开，0.0 表示关闭
        target_end = 1.0 if open else 0.0

        # 在方法内直接创建发布者并发布消息
        gripper_pub = rospy.Publisher('/airbot_play/gripper/set_position', Float64, queue_size=10)
        gripper_command = Float64()
        gripper_command.data = target_end
        gripper_pub.publish(gripper_command)

    # 移动函数
    def home(self):
        """将 Airbot 返回到初始位置"""
        self.move([0, 0, 0, 0, 0, 0])  # 假设关节角度为0代表回家

    # def move(self, input_angles):
    #     """根据输入的关节角度移动 Airbot"""
    #     joint_command = JointState()
    #     joint_command.position = input_angles
    #     self.joint_command_pub.publish(joint_command)
    def move(self, input_angles):
        joint_command_pub = rospy.Publisher('/airbot_play/set_target_joint_q', JointState, queue_size=10)

        joint_command = JointState()
        joint_command.header.stamp = rospy.Time.now()
        joint_command.position = input_angles
        joint_command.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_command.velocity = [0] * len(input_angles)
        joint_command.effort = [0] * len(input_angles)

        rospy.sleep(0.1)
        joint_command_pub.publish(joint_command)




    def move_coords(self, target_translation, target_rotation, velocity=0.2):
        """根据输入的笛卡尔坐标和四元数移动 Airbot"""
        pose_command = Pose()
        pose_command.position.x, pose_command.position.y, pose_command.position.z = target_translation
        pose_command.orientation.x, pose_command.orientation.y, pose_command.orientation.z, pose_command.orientation.w = target_rotation
        self.pose_command_pub.publish(pose_command)