import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import xmlrpclib
import netifaces as ni
from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
import numpy as np
import socket
import time
import json

from rm_config import rm_config 

robot_arm_ip = rm_config["robot_arm"]["robot_arm_ip"]
ip = ni.ifaddresses('eth0')[ni.AF_INET][0]['addr']

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
        
server = SimpleXMLRPCServer((ip,8001),requestHandler=RequestHandler)

conf = rtde_config.ConfigFile("ur5_interface_configuration.xml")
output_names, output_types = conf.get_recipe('out')
rtde_proxy = rtde.RTDE(robot_arm_ip, 30004)
rtde_proxy.connect()
if not rtde_proxy.send_output_setup(output_names, output_types):
    print('wrong config file')
    exit()

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

server.register_function(get_tcp_pose,"get_tcp_pose")
server.register_function(get_tcp_force,"get_tcp_force")
server.register_function(get_joint_angles,"get_joint_angles")
server.register_function(get_robot_status,"get_robot_status")


server.serve_forever()
