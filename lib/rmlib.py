# Import rm_config 
from rm_config import rm_config

import numpy as np
import math
import matplotlib as mpl
from IPython.display import Markdown

# Import initial classes
from intf_viewer_threejs import Viewer
from poses import Poses
from cloud_processing import Cloud_Processing
from cloud_segmentation import Cloud_Segmentation
from cloud_poses import Cloud_Poses
from image_processing import Image_Processing
from intf_xmlrpc_rmlib import XMLRPC_RMLib
from features import Features
from movements import Movements
from other_stuff import Other_Stuff
from intf_urcap import URCap

# Set initial inharitence list
inheritance_list = [Viewer, Poses, Cloud_Processing, Image_Processing, Cloud_Segmentation, Cloud_Poses, XMLRPC_RMLib, Features, Movements, Other_Stuff, URCap]

end_effector_config = None
robot_arm_config = None
force_sensor_config = None
mobile_platform_config = None

# Import End-Effector
if rm_config['import']['end_effector']:
    end_effector_config = rm_config['end_effector']
    # RM SmartHand
    if end_effector_config['type'] == 'smarthand':
        # Import Camera
        if end_effector_config['camera'] == 'realsense_d410':
            from intf_camera_realsense_d410 import RealSense
            inheritance_list.append(RealSense)
        elif end_effector_config['camera'] == 'realsense_d435':
            from intf_camera_realsense_d435 import RealSense
            inheritance_list.append(RealSense)
        else: print('Type: ', end_effector_config['camera'], 'Not Supported')
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
    elif robot_arm_config['type'] == 'ursim':
        from intf_arm_ursim import URSIM
        inheritance_list.append(URSIM)
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


# Import Mobile Platform
if rm_config['import']['mobile_platform']:
    mobile_platform_config = rm_config['mobile_platform']
    if mobile_platform_config['type'] == 'mir100':
        from intf_mobile_platform_mir100 import MiR_Cart
        inheritance_list.append(MiR_Cart)  
    # Add additional options here
    else:
        print('Type: ', mobile_platform_config['type'], 'Not Supported')
        
# Import Saftey Skin

# Build RMLib Class
class RMLib(*inheritance_list):
        
    def __init__(self, output=None):       
        self.rmstudio_path = rm_config['rmstudio_path']
              
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
            
        if robot_arm_config and robot_arm_config['type'] == 'ursim':
            self.arm_max_linear_speed = robot_arm_config['max_linear_speed']
            self.arm_max_linear_accel = robot_arm_config['max_linear_accel']
            self.arm_max_joint_speed = robot_arm_config['max_joint_speed']
            self.arm_max_joint_accel = robot_arm_config['max_joint_accel']

            self.arm_default_linear_speed = robot_arm_config['default_linear_speed']
            self.arm_default_linear_accel = robot_arm_config['default_linear_accel']
            self.arm_default_joint_speed = robot_arm_config['default_joint_speed']
            self.arm_default_joint_accel = robot_arm_config['default_joint_accel']
            URSIM.__init__(self)

        if end_effector_config and end_effector_config['type'] == 'smarthand':
            self.finger_length = end_effector_config['finger_length']
            self.finger_width_outer = end_effector_config['finger_width_outer']
            self.finger_depth = end_effector_config['finger_depth']
            self.tcp_to_camera_pose = self.translate_pose(np.eye(4), x=-0.037, y=-0.033, z=-0.079-self.finger_length) #TELL AUSTIN IF YOU CHANGE THIS VALUE!!! CALIBRATED 2/27/19
            RealSense.__init__(self)
            OpenCM.__init__(self)
            
            # Thermal Check
            motor_temp = self.get_motor_temperature()
            camera_temp = self.get_camera_temperature()
            
            if camera_temp[0] > 60 or camera_temp[1] > 110:
                raise Exception("Camera over heat!")
            if motor_temp[0] > 75 or motor_temp[1] > 75:
                raise Exception("Motor over heat!")
            
        if force_sensor_config and force_sensor_config['type'] == 'onrobot':
            self.force_sensor_ip = force_sensor_config['ip_address']
            OnRobot.__init__(self)
            
        if mobile_platform_config and mobile_platform_config['type'] == 'mir100':
            MiR_Cart.__init__(self)
        
        print('Robot Ready')
        
    def printr(self,string):
        print("\x1b[31m{}\x1b[0m".format(string))
        
    def printb(self,string):
        print("\x1b[34m{}\x1b[0m".format(string))

    def print_clr(self, string, color='green'):
        display(Markdown("<text style=color:{}>{}</text>".format(color, string)))