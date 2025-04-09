import rospy
import numpy as np
import time
from airbot import create_agent
from sensor_msgs.msg import JointState  # 根据需要导入相关消息类型
from geometry_msgs.msg import Pose  # 根据需要导入相关消息类型
from copy import deepcopy as copy



class DexArmControl():
    def __init__(self, record_type=None, robot_type='both'):

        # if pub_port is set to None it will mean that
        # this will only be used for listening to franka and not commanding
        try:
            rospy.init_node("dex_arm", disable_signals=True, anonymous=True)
        except Exception as e:
            rospy.logerr(f"Failed to initialize ROS node: {e}")

        # Create Airbot robot instance
        self.robot = create_agent(can_interface="can0", end_mode="none")  # 创建 Airbot 实例
        self.robot_joint_state = None
        self.robot_commanded_joint_state = None

        self._init_robot_control()

    # Controller initializers
    def _init_robot_control(self):
        # ROS subscribers and publishers
        rospy.Subscriber("/airbot_play/joint_states", JointState, self._callback_robot_joint_state)
        rospy.Subscriber("/airbot_play/set_target_joint_q", JointState, self._callback_robot_commanded_joint_state)
        self.end_pose_pub = rospy.Publisher("/airbot_play/end_pose", Pose, queue_size=10)

    # Rostopic callback functions
    def _callback_robot_joint_state(self, joint_state):
        self.robot_joint_state = joint_state

    def _callback_robot_commanded_joint_state(self, joint_state):
        self.robot_commanded_joint_state = joint_state

    # State information function
    def get_robot_state(self):
        if self.robot_joint_state is None:
            return None

        raw_joint_state = copy(self.robot_joint_state)
        joint_state = dict(
            position = np.array(raw_joint_state.position, dtype = np.float32),
            velocity = np.array(raw_joint_state.velocity, dtype = np.float32),
            effort = np.array(raw_joint_state.effort, dtype = np.float32),
            timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state

    def get_commanded_robot_state(self):
        raw_joint_state = copy(self.robot_commanded_joint_state)
        joint_state = dict(
            position=np.array(raw_joint_state.position, dtype=np.float32),
            velocity=np.array(raw_joint_state.velocity, dtype=np.float32),
            effort=np.array(raw_joint_state.effort, dtype=np.float32),
            timestamp=raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state
    def get_hand_position(self):
            if self.robot_joint_state is None:
                return None

            return np.array(self.robot_joint_state.position, dtype = np.float32)

    def get_hand_velocity(self):
        if self.robot_joint_state is None:
            return None

        return np.array(self.robot_joint_state.velocity, dtype = np.float32)

    def get_hand_torque(self):
        if self.robot_joint_state is None:
            return None

        return np.array(self.robot_joint_state.effort, dtype = np.float32)

    def get_commanded_hand_joint_position(self):
        if self.robot_commanded_joint_state is None:
            return None

        return np.array(self.robot_commanded_joint_state.position, dtype = np.float32)



    def get_arm_cartesian_state(self):
        current_pos, current_quat = copy(self.robot.get_cartesian_position())

        cartesian_state = dict(
            position = np.array(current_pos, dtype=np.float32).flatten(),
            orientation = np.array(current_quat, dtype=np.float32).flatten(),
            timestamp = time.time()
        )

        return cartesian_state


    def get_arm_joint_state(self):
        joint_positions = copy(self.robot.get_joint_position())
        # print('joint_position: {}'.format(joint_positions))

        joint_state = dict(
            position = np.array(joint_positions, dtype=np.float32),
            timestamp = time.time()
        )

        return joint_state
    
    def get_arm_pose(self):
        pose = copy(self.robot.get_pose())

        pose_state = dict(
            position = np.array(pose, dtype=np.float32),
            timestamp = time.time()
        )

        return pose_state

    def get_arm_position(self):

        joint_state = self.get_arm_joint_state()
        return joint_state['position']


    def get_arm_velocity(self):
        raise ValueError('get_arm_velocity() is being called - Arm Velocity cannot be collected in Franka arms, this method should not be called')

    def get_arm_torque(self):
        raise ValueError('get_arm_torque() is being called - Arm Torques cannot be collected in Franka arms, this method should not be called')


    def get_arm_cartesian_coords(self):
        current_pos, current_quat = copy(self.robot.get_cartesian_position())

        current_pos = np.array(current_pos, dtype=np.float32).flatten()
        current_quat = np.array(current_quat, dtype=np.float32).flatten()

        cartesian_coord = np.concatenate(
            [current_pos, current_quat],
            axis=0
        )

        return cartesian_coord
    # Get the robot joint/cartesian position
    def get_robot_position(self):
        return self.robot.get_current_translation()  # 获取机器人当前位置信息

    # Get the robot joint velocity
    def get_robot_velocity(self):
        return self.robot.get_current_joint_v()  # 获取机器人当前关节速度

    # Get the robot joint torque
    def get_robot_torque(self):
        return self.robot.get_current_joint_t()  # 获取机器人当前关节扭矩

    # Get the commanded robot joint position
    def get_commanded_robot_joint_position(self):
        return self.robot.get_current_joint_q()  # 获取命令的关节位置

    # Movement functions
    def move_robot(self, joint_angles):
        if self.robot.valid_joint_q(joint_angles):
            self.robot.set_target_joint_q(joint_angles)  # 使用Airbot库中的方法

    def home_robot(self):
        # 自定义方法使机器人回到初始位置（可根据需要实现具体逻辑）
        self.move_robot([0, 0, 0, 0, 0, 0])  # 假设关节角度为0代表回家

    def reset_robot(self):
        self.robot.reset_error()  # 使用 Airbot 库中的方法来重置错误

    def move_robot_with_arm_angles(self, joint_angles, arm_angles):
        # 假设使用命令来移动到目标关节角度
        self.move_robot(joint_angles)  # 移动关节
        # 对于手臂角度的移动，需要实现相应的逻辑
        # self.robot.set_target_joint_v(arm_angles)  # 如果有对应的手臂角度设置方法

    def arm_control(self, arm_pose):
        # 假设 arm_pose 是一个包含位移和旋转的元组
        self.robot.set_target_pose(arm_pose)  # 使用 Airbot 库中的方法

    def publish_end_pose(self):
        current_pose = self.robot.get_current_pose()  # 获取机器人当前的位姿
        pose_msg = Pose()
        pose_msg.position.x = current_pose[0][0]
        pose_msg.position.y = current_pose[0][1]
        pose_msg.position.z = current_pose[0][2]
        pose_msg.orientation.x = current_pose[1][0]
        pose_msg.orientation.y = current_pose[1][1]
        pose_msg.orientation.z = current_pose[1][2]
        pose_msg.orientation.w = current_pose[1][3]
        self.end_pose_pub.publish(pose_msg)
