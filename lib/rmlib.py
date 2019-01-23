from point_cloud_processing import Point_Cloud_Processing
from segmentation import Segmentation
from grasps import Grasps
from viewer import Viewer
from transforms import Transforms
from config import config

import_realsense = config["imports"]["realsense"]
import_motor_control = config["imports"]["motor_control"]
import_ur5 = config["imports"]["ur5"]
import_optoforce = config["imports"]["optoforce"]
import_binpicking = config["imports"]["binpicking"]
inheritance_list = [Viewer,Transforms,Point_Cloud_Processing,Segmentation,Grasps]

#=================================#
#            IMPORTS              #
#=================================#
    
if import_realsense:
    try:
        from realsense import RealSense
        inheritance_list.append(RealSense)
    except:
        print("\x1b[31m{}\x1b[0m".format("Warning: RealSense not imported successfully"))
              
if import_motor_control:
    try:
        from opencm import OpenCM 
        inheritance_list.append(OpenCM)
    except:
        print("\x1b[31m{}\x1b[0m".format("Warning: Gripper functions not imported successfully"))

if import_ur5:
    try:
        from ur5 import UR5
        inheritance_list.append(UR5) 
    except:
        print("\x1b[31m{}\x1b[0m".format("Warning: UR5 not imported successfully"))                
    
if import_optoforce:
    try:
        from optoforce import OptoForce
        inheritance_list.append(OptoForce)
    except:
        print("\x1b[31m{}\x1b[0m".format("Warning: OptoForce not imported successfully"))
              
if import_binpicking:
    try:
        from binpicking import BinPicking
        inheritance_list.append(BinPicking)
    except:
        print("\x1b[31m{}\x1b[0m".format("Warning: Binpicking package not installed on this hand."))

#=================================#
#            IMPORTS              #
#=================================#
              
class RMLib(*inheritance_list):
        
    def __init__(self,output=0):  
        self.finger_offset = config['offsets']['finger_offset']
        self.camera_offset = [0.0375, 0.032,-0.079-self.finger_offset,0.0,0.0,0.0]
        self.force_sensor_ip = config['force_sensor']['force_sensor_ip']
        self.robot_arm_ip = config['robot_arm']['robot_arm_ip']         
        self.force_sensor = import_optoforce
        self.force_sensor_ip = config['force_sensor']['force_sensor_ip']
                    
        if import_ur5:
            UR5.__init__(self)
            
        if import_realsense:
            RealSense.__init__(self)
            
        if import_optoforce:
            OptoForce.__init__(self)
                   
        if import_motor_control:
            OpenCM.__init__(self)
              
        print("Robot Ready")
        
    def printr(self,string):
        print("\x1b[31m{}\x1b[0m".format(string))
        
    def printb(self,string):
        print("\x1b[34m{}\x1b[0m".format(string))

