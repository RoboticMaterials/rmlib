# NOTE: XMLRPC functions load when the 'start_interfaces' script is run. If you make changes here, the script must be run again
#       in order for the changes to take effect!

import sys
sys.path.append('/home/nvidia/rmstudio/lib')

import rtde
import rtde_config 
import xmlrpclib
import netifaces as ni
from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
import numpy as np
import socket
import time
import json
from transforms import *

from rm_config import rm_config # NOTE: This is "rm/interfaces/config.py" , NOT the PyPI pakckage "config"!

robot_arm_ip = rm_config["robot_arm"]["ip_address"]
robot_proxy_ip = ni.ifaddresses('eth0')[ni.AF_INET][0]['addr']

class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    # Add these headers to all responses
    def end_headers(self):
        self.send_header("Access-Control-Allow-Headers",
        "Origin, X-Requested-With, Content-Type, Accept")
        self.send_header("Access-Control-Allow-Origin", "*")
        SimpleXMLRPCRequestHandler.end_headers(self)
        
server = SimpleXMLRPCServer((robot_proxy_ip,8001),requestHandler=RequestHandler)

conf = rtde_config.ConfigFile("ur5_interface_configuration.xml")
output_names, output_types = conf.get_recipe('out')
rtde_proxy = rtde.RTDE(robot_arm_ip, 30004)
rtde_proxy.connect()
if not rtde_proxy.send_output_setup(output_names, output_types):
    print('wrong config file')
    exit()



def get_camera_transform():
    # NOTE: XMLRPC can only sends lists (1D/2D) of floats and cannot send Numpy arrays, use 'ndarray.tolist()'
    tcp_pose = get_tcp_pose()
    camera_pose = pose_trans( tcp_pose , camera_offset )
    camera_trans = convert_pose_to_transform( camera_pose )
    camera_trans_list = camera_trans.tolist() # NOTE: This is needed, else error: "cannot marshal <type 'numpy.ndarray'> objects"
    return camera_trans_list

def get_tcp_pose():
    rtde_proxy.send_start()
    state = rtde_proxy.receive()
    tcp_pose = state.actual_TCP_pose
    rtde_proxy.send_pause()
    return tcp_pose

def get_joint_angles():
    rtde_proxy.send_start()
    state = rtde_proxy.receive()
    joint_angles = state.actual_q
    rtde_proxy.send_pause()
    return joint_angles

def get_tcp_force():
    rtde_proxy.send_start()
    state = rtde_proxy.receive()
    tcp_pose = state.actual_TCP_pose
    rtde_proxy.send_pause()
    return tcp_pose

def get_robot_status():
    rtde_proxy.send_start()
    state = rtde_proxy.receive()
    rtde_proxy.send_pause()
    return state.robot_status_bits

def get_saftey_mode():
    rtde_proxy.send_start()
    state = rtde_proxy.receive()
    mode_dig = state.safety_status_bits
    rtde_proxy.send_pause()
    return "{:011b}".format(mode_dig)[::-1]

def get_digital_input_bits():
    rtde_proxy.send_start()
    state = rtde_proxy.receive()
    digital = state.actual_digital_input_bits
    rtde_proxy.send_pause()
    return "{:011b}".format(digital)[::-1]


server.register_function(get_tcp_pose,"get_tcp_pose")
server.register_function(get_tcp_force,"get_tcp_force")
server.register_function(get_joint_angles,"get_joint_angles")
server.register_function(get_robot_status,"get_robot_status")
server.register_function(get_saftey_mode,"get_saftey_mode")
server.register_function(get_digital_input_bits,"get_digital_input_bits")


server.serve_forever()
