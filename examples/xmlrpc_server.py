# This example implements a simple XML RPC server, which allows you to trigger
# functionality or read variables from an external computer.
#
# XML RPC can also be used to interface with the hand from
# Polyscope/URScript. An example program is available in xmlrpc_client.urp
import sys
sys.path.append('../lib')

from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler

import netifaces as ni
import rmlib
rm = rmlib.RMLib()

# Address of the ethernet card that the hand listens on
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
        
server = SimpleXMLRPCServer((robot_proxy_ip,8008),requestHandler=RequestHandler)

def listToPose(l):
  assert type(l) is list
  return {'x' : l[0], 'y' : l[1], 'z' : l[2], 'rx' : l[3], 'ry' : l[4], 'rz' : l[5]}


def get_target_pose():
    # Put your algorithm that calculates a pose that the robot should move to here.
    # Your pose should be in the form [x,y,z,rx,ry,rz]. Use "listToPose" to convert
    # the list into a dictionary, which results in the "pose" datatype (as opposed to 
    # joint angles) in URScript. 
    pose = [0.563, -0.114, 0.22, 2.20, 2.223,0]
    return listToPose(pose)

def open_gripper():
    return rm.open_gripper()
    
def close_gripper():
    return rm.close_gripper()
    
def set_gripper_torque(torque):
    return rm.set_gripper_torque(torque) 
    

server.register_function(open_gripper,"open_gripper")
server.register_function(close_gripper,"close_gripper")
server.register_function(set_gripper_torque,"set_gripper_torque")
server.register_function(get_target_pose,"get_target_pose")


server.serve_forever()