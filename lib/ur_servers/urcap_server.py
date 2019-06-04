import sys
sys.path.append('/home/nvidia/rmstudio/lib') 

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

quit=0

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

def run_cmd(dat_string):
    print(dat_string)
    try:
        print(exec(dat_string))
    except SyntaxError as err:
        error_class = err.__class__.__name__
        detail = err.args[0]
        line_number = err.lineno
        print("%s at line %d: %s" % (error_class, line_number, detail))
        time.sleep(0.15) 
        return("%s at line %d: %s" % (error_class, line_number, detail))
    except Exception as err:
        error_class = err.__class__.__name__
        detail = err.args[0]
        cl, exc, tb = sys.exc_info()
        line_number = traceback.extract_tb(tb)[-1][1]
        print("%s at line %d: %s" % (error_class, line_number, detail))
        time.sleep(0.15) # add this so that the GUI can catch the last block at which the error happened
        return("%s at line %d: %s" % (error_class, line_number, detail))
    else: 
        return "ok"

def init():
    try:
        rm
        print("RMLib already loaded")
    except:
        import rmlib
        global rm
        rm = rmlib.RMLib()
    return True

def set_robot_ip(newaddr):
	import os
	from rm_config import rm_config
	oldaddr=rm_config['robot_arm']['robot_arm_ip']
	os.system("sed -i -e 's/'" + oldaddr +"'\\b/'"+newaddr+"'/g' /home/nvidia/rmstudio/lib/rm_config.py")
	print("Replaced " + oldaddr + " with " + newaddr + " in rm_config")
	return True

		
def irimage():
# 	import cv2
# 	import numpy as np
# 	rm.set_laser_state(False)
# 	rm.set_auto_exposure_on(True)
# 	#rm.set_exposure(165000)
	
# 	img = rm.get_ir_image()
# 	img = cv2.flip(img,0)
# 	img = cv2.flip(img,1)
# 	img = np.array(img*255./img.max()).astype(np.uint8)

# 	cv2.imwrite('/media/ramdisk/snapshot.png',img)
    print('hello')
    return True	

def listToPose(l):
    assert type(l) is list
    return {'x' : l[0], 'y' : l[1], 'z' : l[2], 'rx' : l[3], 'ry' : l[4], 'rz' : l[5]}


def close_gripper(force):
	rm.set_gripper_torque(force)
	return_value = rm.close_gripper()
	return bool(return_value)

def get_object_defs():
    try:
        feature_lib = rm.load_feature_lib('/home/nvidia/dev_rmstudio/urcap_server/feature_lib.json')
        string = ''
        for i in feature_lib:
            string = string + i + '%'
        string=string[:-1]
        print("Object definitions requested")
        return string
    except Exception as e: 
        print(e)
        return "generic"

def get_object_pose(feature_name):
    try:
        plane_tol = 0.0045
        feature_lib = rm.load_feature_lib('/home/nvidia/dev_rmstudio/urcap_server/feature_lib.json')
        #Get Cloud
        capture = rm.get_feature_capture(feature_lib[feature_name])

        cloud_full_ds = rm.downsample_cloud(capture['cloud_raw'],leaf_size=0.003)

        #Remove Plane
        capture['cloud_raw'] = rm.remove_planar_surface(capture['cloud_raw'], plane_tol)

        #Find Feature
        trans, clouds_filt, cloud_ds = rm.find_feature(capture, feature_lib[feature_name])

        cld_and_trans = zip(clouds_filt,trans)
        cld_and_trans = sorted(cld_and_trans,key=lambda x: len(x[0]))
        cld_and_trans.reverse()

        cloud_sel = None
        trans_sel = None


        for item in cld_and_trans:
            cloud_sel_width = rm.get_cloud_width(item[0],item[1])
            if cloud_sel_width < 0.1:
                cloud_sel = item[0]
                trans_sel = item[1]
                break

        Tbc = rm.get_base_to_camera_trans()    
        Tco = rm.invert_trans(Tbc).dot(item[1])
        rpy = rm.rotation_matrix_to_RPY(Tco[0:3, 0:3])


        if abs(math.degrees(rpy[2])) > 90:
            trans_sel = rm.rotate_trans(trans_sel,rz=math.radians(180),frame='self')
            trans_sel_new = rm.rotate_trans(trans_sel,rx=rpy[0],ry=rpy[1],frame='self')
        else:
            trans_sel_new = rm.rotate_trans(trans_sel,rx=-rpy[0],ry=-rpy[1],frame='self')

        shift1 = abs(rm.get_distance_estimate(cloud_full_ds)-trans_sel_new[2,3])/2 
        r = 0.055
        shift2 = r-math.sqrt(math.pow(r,2)-math.pow(cloud_sel_width/2,2))
        trans_sel_new = rm.translate_trans(trans_sel_new,z=shift1+shift2, frame='self')
        pose = rm.trans_matrix_to_vec(trans_sel_new)
        print(pose)
        return listToPose(pose)
    except Exception as e: 
        print(e)
        print('here')
        return listToPose([0.1, -0.114, 0.22, 2.20, 2.223,0])
#     try:
#         time.sleep(10)
#         return listToPose(rm.get_tcp_pose())
#     except Exception as e: 
#         print(e)
#         print('here')
#         return listToPose([0.1, -0.114, 0.22, 2.20, 2.223,0])

def get_object_info(object):
	#print("Info requested for object "+object)
	pose = [0.563,-0.114,0.22,2.20,2.223,0]
	width = 0.05
	success = 0
	pose.extend([width])
	pose.extend([success])
	return pose

def open_gripper(force):
	rm.set_gripper_torque(force)
	return_value = rm.open_gripper()
	return bool(return_value)

def set_gripper_width(aperture,force):
	rm.set_gripper_torque(force)
	return_value = rm.set_gripper_width(aperture)
	return bool(return_value)

def stop():
	global quit
	quit=1
	sys.exit("Stop request from user")
	return True

def test():
	pose =[0.563,-0.114,0.22,2.2,2.223,0]
	pose.extend([3.1415]) #add width
	return pose 
	#return {'Pose' : listToPose(pose), 'Width' : 3.1415}

def update():
	import os
	os.system("git pull")
	return True

print("Starting XML-RPC server on port 8101")
server = SimpleXMLRPCServer(("", 8101),requestHandler=RequestHandler)
server.register_introspection_functions()
server.register_function(close_gripper)
#server.register_function(run_cmd)
server.register_function(init)
server.register_function(irimage)
server.register_function(get_object_defs)
server.register_function(get_object_pose)
server.register_function(get_object_info)
server.register_function(open_gripper)
server.register_function(set_gripper_width)
server.register_function(set_robot_ip)
server.register_function(stop)
server.register_function(test)

#server.serve_forever()
while not quit:
	server.handle_request()
