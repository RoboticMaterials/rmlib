# Import rm_config 
from rm_config import rm_config
import numpy as np
import math

# Import initial classes
from intf_viewer_threejs import Viewer
from poses import Poses
from cloud_processing import Cloud_Processing
from cloud_segmentation import Cloud_Segmentation
from cloud_poses import Cloud_Poses
from features import Features
from movements import Movements

# Set initial inharitence list
inheritance_list = [Viewer, Poses, Cloud_Processing, Cloud_Segmentation, Cloud_Poses, Features, Movements]


end_effector_config = None
robot_arm_config = None
force_sensor_config = None

# Import End-Effector
if rm_config['import']['end_effector']:
    end_effector_config = rm_config['end_effector']
    # RM SmartHand
    if end_effector_config['type'] == 'smarthand':
        # Import Camera
        from intf_camera_realsense import RealSense
        inheritance_list.append(RealSense)
        # Import Motor Control
        from intf_gripper_opencm import OpenCM 
        inheritance_list.append(OpenCM)  
    # Add additional options here
    else:
        print('Type: ', end_effector_config['type'], 'Not Supported')
        
# Import Arm
if rm_config['import']['robot_arm']:
    robot_arm_config = rm_config['robot_arm']
    # UR5
    if robot_arm_config['type'] == 'ur5':
        from intf_arm_ur5 import UR5
        inheritance_list.append(UR5)
    # Add additional options here
    else:
        print('Type: ', robot_arm_config['type'], 'Not Supported')
        
# Import Force Sensor
if rm_config['import']['force_sensor']:
    force_sensor_config = rm_config['force_sensor']
    #OnRobot / Optoforce
    if force_sensor_config['type'] == 'onrobot':
        from intf_ftsensor_onrobot import OnRobot
        inheritance_list.append(OnRobot)  
    #Add additional options here
    else:
        print('Type: ', force_sensor_config['type'], 'Not Supported')


#Import Mobile Platform

#Import Saftey Skin

#Build RMLib Class
class RMLib(*inheritance_list):
        
    def __init__(self, output=None):  
              
        if robot_arm_config and robot_arm_config['type'] == 'ur5':
            self.robot_arm_ip = robot_arm_config['ip_address'] 
            self.arm_max_linear_speed = robot_arm_config['max_linear_speed']
            self.arm_max_linear_accel = robot_arm_config['max_linear_accel']
            self.arm_max_joint_speed = robot_arm_config['max_joint_speed']
            self.arm_max_joint_accel = robot_arm_config['max_joint_accel']

            self.arm_default_linear_speed = robot_arm_config['default_linear_speed']
            self.arm_default_linear_accel = robot_arm_config['default_linear_accel']
            self.arm_default_joint_speed = robot_arm_config['default_joint_speed']
            self.arm_default_joint_accel = robot_arm_config['default_joint_accel']
            UR5.__init__(self)

        if end_effector_config and end_effector_config['type'] == 'smarthand':
            self.finger_offset = end_effector_config['finger_offset']
            self.tcp_to_camera_pose = self.translate_pose(np.eye(4), x=-0.037, y=-0.033, z=-0.079-self.finger_offset) #TELL AUSTIN IF YOU CHANGE THIS VALUE!!! CALIBRATED 2/27/19
            RealSense.__init__(self)
            OpenCM.__init__(self)
            
        if force_sensor_config and force_sensor_config['type'] == 'onrobot':
            self.force_sensor_ip = force_sensor_config['ip_address']
            OnRobot.__init__(self)
        
        print("Robot Ready")
        
    def printr(self,string):
        print("\x1b[31m{}\x1b[0m".format(string))
        
    def printb(self,string):
        print("\x1b[34m{}\x1b[0m".format(string))

