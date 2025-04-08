import numpy as np
import matplotlib.pyplot as plt
import zmq
import time
from mpl_toolkits.mplot3d import Axes3D
from tqdm import tqdm

from openteach.utils.network import ZMQKeypointSubscriber,ZMQKeypointPublisher
from openteach.robot.bimanual_airbot_left import AirbotArmLeft
from .operator import Operator
from openteach.utils.timer import FrequencyTimer
from copy import deepcopy as copy
from scipy.spatial.transform import Rotation

prev_position = None
move_threshold = 0.00  # Define position and pose change threshold


ARM_TELEOP_STOP = 0
ARM_TELEOP_CONT = 1
ARM_HIGH_RESOLUTION = 1
ARM_LOW_RESOLUTION = 0

VR_FREQ = 80
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
        # Extract position data (xyz)
        position = matrix_4x3[3, :3]  # Use the first three elements of the last row
        
        # Extract rotation matrix
        rotation_matrix = matrix_4x3[:3, :3]  # Take the first 3 rows and 3 columns as rotation matrix
        
        # Convert the rotation matrix to quaternion
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()  # Quaternion format is [x, y, z, w]
        
        # Return pose containing position and quaternion
        pose = {
            'position': position.tolist(),
            'quaternion': quaternion.tolist()
        }
        return pose
np.set_printoptions(precision=2, suppress=True)


def create_homogeneous_matrix(position, quaternion):
    # position is a list or array containing x, y, z
    if len(position) != 3:
        raise ValueError("Position must have exactly three components (x, y, z).")
    
    # quaternion is a list or array containing quaternion [qw, qx, qy, qz]
    if len(quaternion) != 4:
        raise ValueError("Quaternion must have exactly four components [qw, qx, qy, qz].")
    
    # Generate rotation matrix from quaternion
    rotation = Rotation.from_quat(quaternion)  # Using scipy's Rotation class
    rotation_matrix = rotation.as_matrix()  # Get 3x3 rotation matrix
    
    # Create a 4x4 homogeneous transformation matrix
    homo_matrix = np.eye(4)  # Initialize as identity matrix
    homo_matrix[:3, :3] = rotation_matrix  # Fill in rotation matrix
    homo_matrix[:3, 3] = position  # Put position in the last column

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


# Airbot operator class
class AirbotLeftOperator(Operator):
    def __init__(
        self,
        host,
        transformed_keypoints_port,
        use_filter=False,
        arm_resolution_port = None,
        # teleoperation_reset_port = None,
        gripper_port=None,
        cartesian_publisher_port = None,
        joint_publisher_port = None,
        cartesian_command_publisher_port = None
    ):
        self.notify_component_start('airbot operator')
        # Subscriber for transformed hand keypoints
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
                # Gripper and cartesian publisher
        self.gripper_publisher = ZMQKeypointPublisher(
            host=host,
            port=gripper_port
        )

        self.cartesian_publisher = ZMQKeypointPublisher(
            host=host,
            port=cartesian_publisher_port
        )

        self.joint_publisher = ZMQKeypointPublisher(
            host=host,
            port=joint_publisher_port
        )

        self.cartesian_command_publisher = ZMQKeypointPublisher(
            host=host,
            port=cartesian_command_publisher_port
        )    

        # Initialize robot controller
        self._robot = AirbotArmLeft()
        self.resolution_scale = 1  # Default resolution scale
        self.arm_teleop_state = ARM_TELEOP_STOP  # Default state is stopped
        self.gripper_correct_state =0
        self.pause_flag=0
        self.prev_gripper_flag=0
        self.gripper_flag=1
        self.pause_cnt=0

        # Subscriber for resolution and teleop state
        self._arm_resolution_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = arm_resolution_port,
            topic = 'button'
        )

        # self._arm_teleop_state_subscriber = ZMQKeypointSubscriber(
        #     host = host, 
        #     port = teleoperation_reset_port,
        #     topic = 'pause'
        # )

        # Robot initial position
        self.robot_init_position = self.robot.get_cartesian_state()
        self.is_first_frame = True

        self.use_filter = use_filter
        if use_filter:
            robot_init_cart = self._homo2cart(self.robot.get_cartesian_position())
            self.comp_filter = Filter(robot_init_cart, comp_ratio=0.8)

        self._timer = FrequencyTimer(VR_FREQ)

        # Class Variables
        self.resolution_scale =1
        self.arm_teleop_state = ARM_TELEOP_STOP
        self.is_first_frame= True
        self.prev_gripper_flag=0
        self.prev_pause_flag=0
        self.pause_cnt=0
        self.gripper_correct_state=1
        self.gripper_flag=1
        self.pause_flag=1
        self.gripper_cnt=0


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
    
    # Get the arm teleop state from the hand keypoints
    def _get_arm_teleop_state_from_hand_keypoints(self):
        pause_state ,pause_status,pause_left =self.get_pause_state_from_hand_keypoints()
        pause_status =np.asanyarray(pause_status).reshape(1)[0] 
        return pause_state,pause_status,pause_left
    
    # Get the resolution scale mode (High or Low)
    def _get_resolution_scale_mode(self):
        data = self._arm_resolution_subscriber.recv_keypoints()
        res_scale = np.asanyarray(data).reshape(1)[0] # Make sure this data is one dimensional
        return res_scale  

    # # Get the teleop state (Pause or Continue)
    # def _get_arm_teleop_state(self):
    #     reset_stat = self._arm_teleop_state_subscriber.recv_keypoints()
    #     reset_stat = np.asanyarray(reset_stat).reshape(1)[0] # Make sure this data is one dimensional
    #     return reset_stat

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
        # Check if the operation state needs to be reset
        new_arm_teleop_state,pause_status,pause_left = self._get_arm_teleop_state_from_hand_keypoints()
        if new_arm_teleop_state == ARM_TELEOP_STOP:
            print("Left arm is stopped")
            return  # Return directly, no updates
        if self.is_first_frame or (self.arm_teleop_state == ARM_TELEOP_STOP and new_arm_teleop_state == ARM_TELEOP_CONT):
            self._reset_teleop()  # Reset operation
            moving_hand_frame = None
        else:
            moving_hand_frame = self._get_hand_frame()  # Get current hand frame
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
        # If hand frame is not obtained, return
        if moving_hand_frame is None:
            return
        # print(self.gripper_correct_state)
        self.hand_moving_H = self._turn_frame_to_homo_mat(moving_hand_frame)

        # Transformation code
        H_HI_HH = copy(self.hand_init_H) # Homogeneous matrix from "hand initial position (HI)" to "hand current position (HH)"
        H_HT_HH = copy(self.hand_moving_H) # Homogeneous matrix from "hand current frame (HT)" to "hand current position (HH)"
        # Extract position part (first three elements)
        H_RI_RH_position = self.robot_init_position[0] 
        H_RI_RH_quaternion = self.robot_init_position[1] 
        H_RI_RH = create_homogeneous_matrix(H_RI_RH_position,H_RI_RH_quaternion)  # Transformation from "robot initial position (RI)" to "robot current position (RH)"
        rotation_matrix_ccw = np.array([
            [0, -1, 0, 0],
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Clockwise rotation 90 degrees
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
                [0,0,0,1]])# Align the hand coordinate system to the robot coordinate system  

        H_HT_HI = np.linalg.pinv(H_HI_HH) @ H_HT_HH # Transformation from "hand current frame (HT)" to "hand initial frame (HI)"
        H_RT_RH = H_RI_RH @ H_A_R @ H_HT_HI @ np.linalg.pinv(H_A_R) # How the robot moves from current state (RT) to target position (RH)
        self.robot_moving_H = copy(H_RT_RH)

        # Use the resolution scale to get the final cart pose
        final_pose = self._get_scaled_cart_pose(self.robot_moving_H)

        # If using filter, apply filtering
        if self.use_filter:
            final_pose = self.comp_filter(final_pose)
        # self.gripper_publisher.pub_keypoints(self.gripper_correct_state,"gripper_left")
        # position=self.robot.get_cartesian_position()
        # joint_position= self.robot.get_joint_position()
        # self.cartesian_publisher.pub_keypoints(position,"cartesian")
        # self.joint_publisher.pub_keypoints(joint_position,"joint")
        # self.cartesian_command_publisher.pub_keypoints(final_pose,"cartesian")
        # print(final_pose)

        position = final_pose[:3] 
        rotation = final_pose[3:] 

        current_time = time.time()
        if prev_position is None:
            prev_position = position
        else:
            position_np = np.array(position)
            prev_position_np = np.array(prev_position)

            if np.linalg.norm(position_np - prev_position_np) > move_threshold:
                # print(f"Time: {current_time}, Moving to position: {position}")
                self.robot.move_coords(position, rotation, velocity=0.5)
                prev_position = position


        if gripper_state == GRIPPER_OPEN:
            self.robot.move_gripper(open=True)  # Assuming move_gripper function controls opening the gripper
        elif gripper_state == GRIPPER_CLOSE:
            self.robot.move_gripper(open=False)  # Controls closing the gripper


    def stream(self):
        # Define target position
        target_position = [0.0, -0.8, 0.8, 1.5707, -1.5707, -1.5707]
        self.robot.move(target_position)
        self.notify_component_start('{} control'.format(self.robot.name))
        print("Start controlling the AirbotArm using the Oculus Headset.\n")
        # print(self.robot.get_cartesian_state())
    
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
