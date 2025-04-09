import numpy as np
import matplotlib.pyplot as plt
import zmq
import time
from mpl_toolkits.mplot3d import Axes3D
from tqdm import tqdm

from openteach.utils.network import ZMQKeypointSubscriber
from openteach.robot.airbot_ros import AirbotArm
from .operator import Operator
from openteach.utils.timer import FrequencyTimer
from copy import deepcopy as copy
from scipy.spatial.transform import Rotation

prev_position = None
move_threshold = 0.01  # 定义位置和姿态的变化阈值


ARM_TELEOP_STOP = 0
ARM_TELEOP_CONT = 1
ARM_HIGH_RESOLUTION = 1
ARM_LOW_RESOLUTION = 0

VR_FREQ = 30 
GRIPPER_OPEN=1
GRIPPER_CLOSE=0

OCULUS_JOINTS = {
    'metacarpals': [2, 6, 9, 12, 15],
    'knuckles': [6, 9, 12, 16],
    'thumb': [2, 3, 4, 5, 19],
    'index': [6, 7, 8, 20],
    'middle': [9, 10, 11, 21],
    'ring': [12, 13, 14, 22],
    'pinky': [15, 16, 17, 18, 23]
}
def convert_to_pose(matrix_4x3):
        # 提取位置数据 (xyz)
        position = matrix_4x3[3, :3]  # 使用最后一行的前三个元素
        
        # 提取旋转矩阵
        rotation_matrix = matrix_4x3[:3, :3]  # 取前3行3列作为旋转矩阵
        
        # 将旋转矩阵转化为四元数
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()  # 四元数格式为 [x, y, z, w]
        
        # 返回包含位置和四元数的姿态
        pose = {
            'position': position.tolist(),
            'quaternion': quaternion.tolist()
        }
        return pose
np.set_printoptions(precision=2, suppress=True)


def create_homogeneous_matrix(position, quaternion):
    # position 是一个包含 x, y, z 的列表或数组
    if len(position) != 3:
        raise ValueError("Position must have exactly three components (x, y, z).")
    
    # quaternion 是一个包含四元数 [qw, qx, qy, qz] 的列表或数组
    if len(quaternion) != 4:
        raise ValueError("Quaternion must have exactly four components [qw, qx, qy, qz].")
    
    # 通过四元数生成旋转矩阵
    rotation = Rotation.from_quat(quaternion)  # 使用 scipy 的 Rotation 类
    rotation_matrix = rotation.as_matrix()  # 得到 3x3 旋转矩阵
    
    # 创建一个 4x4 的齐次变换矩阵
    homo_matrix = np.eye(4)  # 初始化为单位矩阵
    homo_matrix[:3, :3] = rotation_matrix  # 填入旋转矩阵
    homo_matrix[:3, 3] = position  # 将位置填入最后一列

    return homo_matrix



# Filter to smooth out the arm cartesian state
class Filter:
    def __init__(self, state, comp_ratio=0.6):
        self.pos_state = state[:3]
        self.ori_state = state[3:7]
        self.comp_ratio = comp_ratio

    def __call__(self, next_state):
        self.pos_state = self.pos_state * self.comp_ratio + next_state[:3] * (1 - self.comp_ratio)
        ori_interp = Rotation.slerp([0, 1], Rotation.from_quat(np.stack([self.ori_state, next_state[3:7]], axis=0)))
        self.ori_state = ori_interp([1 - self.comp_ratio])[0].as_quat()
        return np.concatenate([self.pos_state, self.ori_state])


# Airbot操作类
class AirbotOperator(Operator):
    def __init__(
        self,
        host,
        transformed_keypoints_port,
        use_filter=False,
        arm_resolution_port = None,
        teleoperation_reset_port = None,
    ):
        self.notify_component_start('airbot operator')
        # 订阅器，用于获取变换后的手部关键点
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_keypoints_port,
            topic='transformed_hand_coords'
        )
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_keypoints_port,
            topic='transformed_hand_frame'
        )

        # 初始化机器人控制器
        self._robot = AirbotArm()
        self.resolution_scale = 1  # 默认分辨率比例
        self.arm_teleop_state = ARM_TELEOP_STOP  # 默认状态为停止
        self.gripper_correct_state =0
        self.pause_flag=0
        self.prev_gripper_flag=0
        self.gripper_flag=1
        self.pause_cnt=0

        # 订阅器，用于获取分辨率和遥控状态
        self._arm_resolution_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = arm_resolution_port,
            topic = 'button'
        )

        self._arm_teleop_state_subscriber = ZMQKeypointSubscriber(
            host = host, 
            port = teleoperation_reset_port,
            topic = 'pause'
        )

        # 机器人初始位置
        self.robot_init_position = self.robot.get_cartesian_state()
        self.is_first_frame = True

        self.use_filter = use_filter
        if use_filter:
            robot_init_cart = self._homo2cart(self.robot.get_cartesian_position())
            self.comp_filter = Filter(robot_init_cart, comp_ratio=0.8)

        self._timer = FrequencyTimer(VR_FREQ)

    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return self._robot

    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber
    
    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._transformed_arm_keypoint_subscriber

    # Get the hand frame
    def _get_hand_frame(self):
        for i in range(10):
            data = self.transformed_arm_keypoint_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
            if not data is None: break 
        if data is None: return None
        return np.asanyarray(data).reshape(4, 3)
    
    # Get the resolution scale mode (High or Low)
    def _get_resolution_scale_mode(self):
        data = self._arm_resolution_subscriber.recv_keypoints()
        res_scale = np.asanyarray(data).reshape(1)[0] # Make sure this data is one dimensional
        return res_scale  

    # Get the teleop state (Pause or Continue)
    def _get_arm_teleop_state(self):
        reset_stat = self._arm_teleop_state_subscriber.recv_keypoints()
        reset_stat = np.asanyarray(reset_stat).reshape(1)[0] # Make sure this data is one dimensional
        return reset_stat

    # Converts a frame to a homogenous transformation matrix
    def _turn_frame_to_homo_mat(self, frame):
        t = frame[0]
        R = frame[1:]

        homo_mat = np.zeros((4, 4))
        homo_mat[:3, :3] = np.transpose(R)
        homo_mat[:3, 3] = t
        homo_mat[3, 3] = 1

        return homo_mat
    
    # Converts Homogenous Transformation Matrix to Cartesian Coords
    def _homo2cart(self, homo_mat):
        
        t = homo_mat[:3, 3]
        R = Rotation.from_matrix(
            homo_mat[:3, :3]).as_quat()

        cart = np.concatenate(
            [t, R], axis=0
        )

        return cart
    
    # Gets the Scaled Resolution pose
    def _get_scaled_cart_pose(self, moving_robot_homo_mat):
        # Get the cart pose without the scaling
        unscaled_cart_pose = self._homo2cart(moving_robot_homo_mat)

        cartesian_state = self.robot.get_cartesian_state()
        position = cartesian_state[0]
        quaternion = [0,0,0,1]
        current_homo_mat = copy(create_homogeneous_matrix(position,quaternion))
        current_cart_pose = self._homo2cart(current_homo_mat)

        # Get the difference in translation between these two cart poses
        diff_in_translation = unscaled_cart_pose[:3] - current_cart_pose[:3]
        scaled_diff_in_translation = diff_in_translation * self.resolution_scale
        # print('SCALED_DIFF_IN_TRANSLATION: {}'.format(scaled_diff_in_translation))
        
        scaled_cart_pose = np.zeros(7)
        scaled_cart_pose[3:] = unscaled_cart_pose[3:] # Get the rotation directly
        scaled_cart_pose[:3] = current_cart_pose[:3] + scaled_diff_in_translation # Get the scaled translation only

        return scaled_cart_pose

    # Reset the teleoperation and get the first frame
    def _reset_teleop(self):
        # Just updates the beginning position of the arm
        print('****** RESETTING TELEOP ****** ')
        self.robot_init_position = self.robot.get_cartesian_state()
        first_hand_frame = self._get_hand_frame()
        while first_hand_frame is None:
            first_hand_frame = self._get_hand_frame()
        self.hand_init_H = self._turn_frame_to_homo_mat(first_hand_frame)
        self.hand_init_t = copy(self.hand_init_H[:3, 3])
        self.is_first_frame = False
        return first_hand_frame


    def get_pause_state_from_hand_keypoints(self):
        transformed_hand_coords= self.transformed_hand_keypoint_subscriber.recv_keypoints()
        ring_distance = np.linalg.norm(transformed_hand_coords[OCULUS_JOINTS['ring'][-1]]- transformed_hand_coords[OCULUS_JOINTS['thumb'][-1]])
        middle_distance = np.linalg.norm(transformed_hand_coords[OCULUS_JOINTS['middle'][-1]]- transformed_hand_coords[OCULUS_JOINTS['thumb'][-1]])
        thresh = 0.04 
        pause_right= True
        if ring_distance < thresh or middle_distance < thresh:
            self.pause_cnt+=1
            if self.pause_cnt==1:
                self.prev_pause_flag=self.pause_flag
                self.pause_flag = not self.pause_flag       
        else:
            self.pause_cnt=0
        pause_state = np.asanyarray(self.pause_flag).reshape(1)[0]
        pause_status= False  
        if pause_state!= self.prev_pause_flag:
            pause_status= True 
        return pause_state , pause_status , pause_right
    def get_gripper_state_from_hand_keypoints(self):
        transformed_hand_coords= self.transformed_hand_keypoint_subscriber.recv_keypoints()
        pinky_distance = np.linalg.norm(transformed_hand_coords[OCULUS_JOINTS['pinky'][-1]]- transformed_hand_coords[OCULUS_JOINTS['thumb'][-1]])
        thresh = 0.03
        gripper_fr =False
        if pinky_distance < thresh:
            self.gripper_cnt+=1
            if self.gripper_cnt==1:
                self.prev_gripper_flag = self.gripper_flag
                self.gripper_flag = not self.gripper_flag 
                gripper_fl=True
        else: 
            self.gripper_cnt=0

        gripper_state = np.asanyarray(self.gripper_flag).reshape(1)[0]
        status= False  
        if gripper_state!= self.prev_gripper_flag:
            status= True
        return gripper_state , status , gripper_fr
    
    # Apply the retargeted angles
    def _apply_retargeted_angles(self, log=False):
        global prev_position
        # 检查是否需要重置操作状态
        new_arm_teleop_state = self._get_arm_teleop_state()
        if self.is_first_frame or (self.arm_teleop_state == ARM_TELEOP_STOP and new_arm_teleop_state == ARM_TELEOP_CONT):
            self._reset_teleop()  # 重置操作
            moving_hand_frame = None
        else:
            moving_hand_frame = self._get_hand_frame()  # 获取当前手部框架
        self.arm_teleop_state = new_arm_teleop_state
        # print(moving_hand_frame)
        arm_teleoperation_scale_mode = self._get_resolution_scale_mode()
        if arm_teleoperation_scale_mode == ARM_HIGH_RESOLUTION:
            self.resolution_scale = 1
        elif arm_teleoperation_scale_mode == ARM_LOW_RESOLUTION:
            self.resolution_scale = 0.6

        gripper_state,status_change, gripper_flag = self.get_gripper_state_from_hand_keypoints()
        if self.gripper_cnt==1 and status_change is True:
            self.gripper_correct_state= gripper_state
        # if status_change is True:
        if self.gripper_correct_state == GRIPPER_OPEN:
            gripper_state = 1
        elif self.gripper_correct_state == GRIPPER_CLOSE:
            gripper_state = 0
        # 若未获取到手部框架，返回
        if moving_hand_frame is None:
            return
        # print(self.gripper_correct_state)
        self.hand_moving_H = self._turn_frame_to_homo_mat(moving_hand_frame)

        # Transformation code
        H_HI_HH = copy(self.hand_init_H) #从“手部初始位置（HI）”转换到“手部当前位置（HH）”的齐次矩阵
        H_HT_HH = copy(self.hand_moving_H) #从“手部当前帧（HT）”转换到“手部当前位置（HH）”的齐次矩阵
        # 提取位置部分 (前三个元素)
        H_RI_RH_position = self.robot_init_position[0] 
        H_RI_RH_quaternion = self.robot_init_position[1] 
        H_RI_RH = create_homogeneous_matrix(H_RI_RH_position,H_RI_RH_quaternion)  #从“机器人初始位置（RI）”到“机器人当前位置（RH）”的转换
        rotation_matrix_ccw = np.array([
            [0, -1, 0, 0],
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # 顺时针旋转 90 度
        rotation_matrix_cw = np.array([
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        rotation_matrix_y_90 = np.array([
            [0, 0, 1, 0],
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]
        ])
        rotation_matrix_y_180 = np.array([
            [-1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1]
        ])


        H_A_R = rotation_matrix_y_180@rotation_matrix_y_90@rotation_matrix_cw@np.array( 
                [[1,0,0,0],
                [0,0,1,0],
                [0,-1,0,-0.06],
                [0,0,0,1]])
        H_HT_HI = np.linalg.pinv(H_HI_HH) @ H_HT_HH 
        H_RT_RH = H_RI_RH @ H_A_R @ H_HT_HI @ np.linalg.pinv(H_A_R)
        self.robot_moving_H = copy(H_RT_RH)

        final_pose = self._get_scaled_cart_pose(self.robot_moving_H)

        # 若使用滤波器，应用滤波
        if self.use_filter:
            final_pose = self.comp_filter(final_pose)

        print(final_pose)

        position = final_pose[:3] 
        rotation = final_pose[3:] 

        current_time = time.time()
        if prev_position is None:
            prev_position = position
        else:
            position_np = np.array(position)
            prev_position_np = np.array(prev_position)

            if np.linalg.norm(position_np - prev_position_np) > move_threshold:
                print(f"Time: {current_time}, Moving to position: {position}")
                self.robot.move_coords(position, rotation, velocity=0.3)
                prev_position = position

        if gripper_state == GRIPPER_OPEN:
            self.robot.move_gripper(open=True)  
        elif gripper_state == GRIPPER_CLOSE:
            self.robot.move_gripper(open=False) 

    def stream(self):

            target_position = [0.0, -0.8, 0.8, 1.5707, -1.5707, -1.5707]
            self.robot.move(target_position)
            self.notify_component_start('{} control'.format(self.robot.name))
            print("Start controlling the AirbotArm using the Oculus Headset.\n")
            print(self.robot.get_cartesian_state())
            self.robot.move(target_position)
        
            while True:
                try:
                    if self.robot.get_cartesian_state() is not None:
                        self.timer.start_loop()

                        # Retargeting function
                        self._apply_retargeted_angles(log=False)

                        self.timer.end_loop()
                except KeyboardInterrupt:
                    break

            self.transformed_arm_keypoint_subscriber.stop()
            print('Stopping the teleoperator!')