import sys
import os.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

from xmlrpc.server import SimpleXMLRPCServer
import xmlrpc.client
from xmlrpc.server import SimpleXMLRPCRequestHandler
import time
import base64
import netifaces as ni
import numpy as np
import sys
import traceback
import math
from IPython.display import clear_output
import subprocess
from rm_config import rm_config

quit=0
feature_lib_path = rm_config['rmstudio_path']+'lib/ur_servers/urcap_feature_lib.json'

class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2')

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    # Add these headers to all responses
    def end_headers(self):
        self.send_header("Access-Control-Allow-Headers",
        "Origin, X-Requested-With, Content-Type, Accept")
        self.send_header("Access-Control-Allow-Origin", "*")
        SimpleXMLRPCRequestHandler.end_headers(self)

def set_robot_ip(newaddr):
	import os
	from rm_config import rm_config
	oldaddr=rm_config['robot_arm']['ip_address']
	os.system("sed -i -e 's/'" + oldaddr +"'\\b/'"+newaddr+"'/g' "+rm_config['rmstudio_path']+"lib/rm_config.py")
	print("Replaced " + oldaddr + " with " + newaddr + " in rm_config")
	return True

def init():
    print('start init')
    global urcap_kernel
    try:
       urcap_kernel
    except:
       urcap_kernel=subprocess.Popen(['python3','run_urcap_kernel.py'])
       time.sleep(0.5)
    return True
    
def stop():
	urcap_kernel.terminate()
	global quit
	quit=1
	return True

# def listToPose(l):
#     assert type(l) is list
#     return {'x' : l[0], 'y' : l[1], 'z' : l[2], 'rx' : l[3], 'ry' : l[4], 'rz' : l[5]}

# def get_object_defs():
#     try:
#         feature_lib = rm.load_feature_lib(feature_lib_path)
#         string = ''
#         for i in feature_lib:
#             string = string + i + '%'
#         string=string[:-1]
#         print("Object definitions requested")
#         return string
#     except Exception as e: 
#         print(e)
#         return "generic"

# def get_object_pose_base(feature_name):
#     plane_tol = 0.0045
#     feature_lib = rm.load_feature_lib(feature_lib_path)

#     #Get Cloud
#     rm.set_disparity_shift(100)
#     capture = rm.get_feature_capture(feature_lib[feature_name])

#     #Downsample Cloud
#     cloud_full_ds = rm.downsample_cloud(capture['cloud_raw'],leaf_size=0.003)

#     #Remove Plane
#     capture['cloud_raw'] = rm.remove_planar_surface(capture['cloud_raw'], plane_tol, rmv_high = False, rmv_low = True)

#     #Find Feature
#     trans, clouds_filt, cloud_ds = rm.find_feature(capture, feature_lib[feature_name])

#     #Zip clouds and trans
#     cld_and_trans = zip(clouds_filt,trans)

#     #Sort largest to smallest
#     cld_and_trans = sorted(cld_and_trans,key=lambda x: len(x[0]))
#     cld_and_trans.reverse()

#     #Find the first cloud within the grippers width
#     cloud_sel = None
#     trans_sel = None
#     for item in cld_and_trans:
#         cloud_sel_width = rm.get_cloud_width(item[0],item[1])
#         if cloud_sel_width < 0.1:
#             cloud_sel = item[0]
#             trans_sel = item[1]
#             grasp_pose = rm.align_pose_to_tcp(trans_sel)
#             grasp_pose = rm.shift_pose_to_grasp(cloud_full_ds,grasp_pose,cloud_sel_width,step_size=0.001)
#             if grasp_pose is not None:
#                 break

#     return rm.pose_mtrx_to_vec(grasp_pose), cloud_sel_width*1.5

# def get_object_pose(feature_name):
#     pose, width = get_object_pose_base(feature_name)
#     return listToPose(pose)
    
# def get_object_info(feature_name):
# 	pose, width = get_object_pose_base(feature_name)
# 	width = width.tolist()
# 	success = 1
# 	pose.append(width)
# 	pose.append(success)
# 	return pose

# def open_gripper(force):
# 	proxy.set_gripper_torque(force)
# 	return_value = proxy.open_gripper()
# 	return bool(return_value)

# def close_gripper(force):
# 	proxy.set_gripper_torque(force)
# 	return_value = proxy.close_gripper()
# 	return bool(return_value)

# def set_gripper_width(aperture,force):
# 	proxy.set_gripper_torque(force)
# 	return_value = proxy.set_gripper_width(aperture)
# 	return bool(return_value)

# def test():
# 	pose =[0.563,-0.114,0.22,2.2,2.223,0]
# 	pose.extend([3.1415]) #add width
# 	return pose 
# 	#return {'Pose' : listToPose(pose), 'Width' : 3.1415}
    


print("Starting XML-RPC server on port 8101")
server = SimpleXMLRPCServer(("", 8101),requestHandler=RequestHandler)
server.register_introspection_functions()
server.register_function(init)
server.register_function(stop)
server.register_function(set_robot_ip)

# server.register_function(open_gripper)
# server.register_function(close_gripper)
# server.register_function(get_object_defs)
# server.register_function(get_object_pose)
# server.register_function(get_object_info)
# server.register_function(set_gripper_width)
# server.register_function(test)
#server.register_function(run_cmd)

#server.serve_forever()
while not quit:
	server.handle_request()
sys.exit("Stop request from user")