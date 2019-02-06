#Import and assign rm_config 
from rm_config import rm_config

import_ur5 = rm_config["imports"]["ur5"]
import_realsense = rm_config["imports"]["realsense"]
import_optoforce = rm_config["imports"]["optoforce"]
import_gripper = rm_config["imports"]["motor_control"]
import_assembly = rm_config["imports"]["assembly"]
import_binpicking = rm_config["imports"]["binpicking"]

from viewer import Viewer
from transforms import Transforms
from point_cloud_processing import Point_Cloud_Processing
from segmentation import Segmentation
from grasps import Grasps

#Set initial inharitence list
inheritance_list = [Viewer, Transforms, Point_Cloud_Processing, Segmentation, Grasps]

#Import and add packages to inharitence list
if import_ur5:
    from ur5 import UR5
    inheritance_list.append(UR5) 

if import_realsense:
    from realsense import RealSense
    inheritance_list.append(RealSense)
              
if import_gripper:
    from opencm import OpenCM 
    inheritance_list.append(OpenCM) 
    
if import_optoforce:
    from optoforce import OptoForce
    inheritance_list.append(OptoForce)

if import_assembly:
    from assembly import Assembly
    inheritance_list.append(Assembly)
        
if import_binpicking:
    from binpicking import BinPicking
    inheritance_list.append(BinPicking)

class RMLib(*inheritance_list):
        
    def __init__(self,output=0):  
        self.finger_offset = rm_config['offsets']['finger_offset']
        self.camera_offset = [0.0375, 0.032,-0.079-self.finger_offset,0.0,0.0,0.0]
        
        self.robot_arm_ip = rm_config['robot_arm']['robot_arm_ip']         
        self.force_sensor_ip = rm_config['force_sensor']['force_sensor_ip']
                    
        if import_ur5:
            UR5.__init__(self)
            
        if import_realsense:
            RealSense.__init__(self)
            
        if import_optoforce:
            OptoForce.__init__(self)
                   
        if import_gripper:
            OpenCM.__init__(self)
              
        if import_binpicking:
            BinPicking.__init__(self)
        
        print("Robot Ready")
        
    def printr(self,string):
        print("\x1b[31m{}\x1b[0m".format(string))
        
    def printb(self,string):
        print("\x1b[34m{}\x1b[0m".format(string))



