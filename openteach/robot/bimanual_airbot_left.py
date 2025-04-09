from airbot import create_agent  # 导入 Airbot 库
from .robot import RobotWrapper
import numpy as np

class AirbotArmLeft(RobotWrapper):
    def __init__(self, record_type=None):
        """初始化 AirbotArm控制器"""
        self._robot = create_agent(can_interface="can0", end_mode="gripper")  # 创建 Airbot 实例
        self._data_frequency = 60

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

    # 状态信息函数
    def get_joint_state(self):
        return self._robot.get_current_joint_state()  # 获取当前关节状态

    def get_cartesian_state(self):
        return self._robot.get_current_pose()  # 获取当前笛卡尔位姿

    def get_joint_position(self):
        return self._robot.get_current_joint_q()  # 获取当前关节位置

    def get_joint_velocity(self):
        return self._robot.get_current_joint_v()  # 获取当前关节速度

    def get_joint_torque(self):
        return self._robot.get_current_joint_t()  # 获取当前关节扭矩

    def get_cartesian_position(self):
        return self._robot.get_current_translation()  # 获取当前笛卡尔坐标
    
    def move_gripper(self, open):
        target_end = 1.0 if open else 0.0

        success = self._robot.set_target_end(target_end)
        if not success:
            print("Failed to move the gripper.")
        return success

    # 移动函数
    def home(self):
        """将 Airbot 返回到初始位置"""
        self._robot.reset_error()  # 重置机器人状态
        self.move([0, 0, 0, 0, 0, 0])  # 假设关节角度为0代表回家
        # [0.0, -0.7999999999999999, 0.7999999999999999, 1.5707, -1.5707, -1.5707]

    def move(self, input_angles):
        """根据输入的关节角度移动 Airbot"""
        if self._robot.valid_joint_q(input_angles):
            self._robot.set_target_joint_q(input_angles, vel=0.1,blocking=True)  # 设置目标关节角度

    def move_coords(self, target_translation, target_rotation, velocity=0.2):
        """根据输入的笛卡尔坐标和四元数移动 Airbot"""
        # 调用设置目标姿态的方法，传递位置、四元数和速度
        success = self._robot.set_target_pose(
            target_translation,
            target_rotation,
            vel=velocity,use_planning=False
        )  

        if not success:
            print("Failed to move the robot.")

