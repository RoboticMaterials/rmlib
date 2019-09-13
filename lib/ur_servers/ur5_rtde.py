# PYTHON 2.7

import sys
print( "Python Version:" , sys.version )
import os.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

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

_ENABLE_LOG = 0

if _ENABLE_LOG:
    from ur_server_utils import create_log_file , nowTimeStampFine
    logPath = os.path.join( os.path.expanduser('~') , "rmstudio/" , "notebooks/Sandbox_James/assembly_demos/" , "logs/" )
    _LOG = create_log_file( logPath , "Server-Create-Log_" )
    
    def log_print( *args ):
        """ Debug logging function """
        logStr = ""
        for ar in args:
            print ar ,
            logStr += ' ' + str( ar )
        print
        logStr += '\n'
        _LOG.write( logStr )
        
    def log_end():
        """ Cease logging by closing file """
        _LOG.write( "\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LOG END ~~~" + '\n\n' )
        _LOG.close()

if _ENABLE_LOG:
    _LOG.write( "TIME: ......... " + nowTimeStampFine() + '\n' )
    log_print( "ni.ifaddresses('eth0')[" , ni.AF_INET , "][0]['addr']" )
    
    
# Load Config
from rm_config import rm_config 

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

        
# ~~~ ATTEMPT CONNECTION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
_MAX_ATTEMPT = 10
connectOK    = False
err          = [ False , None ]

for i in range( _MAX_ATTEMPT ):
    
    if _ENABLE_LOG:
        log_print( "~~~" , "Connection attempt" , i+1 , "of" , _MAX_ATTEMPT , "~~~" , '\n' )
    
    try:
        robot_arm_ip   = rm_config["robot_arm"]["ip_address"]
        robot_proxy_ip = ni.ifaddresses('eth0')[ni.AF_INET][0]['addr']        

        if _ENABLE_LOG:
            log_print( "robot_arm_ip:  " , str( robot_arm_ip ) )
            log_print( "robot_proxy_ip:" , str( robot_proxy_ip ) )

        try:
            server = SimpleXMLRPCServer( ( robot_proxy_ip , 8001 ) ,
                                         requestHandler = RequestHandler )
            if _ENABLE_LOG:
                log_print( "STARTED `SimpleXMLRPCServer`!" )
        except Exception as ex:
            if _ENABLE_LOG:
                log_print( "Encountered a problem while starting `SimpleXMLRPCServer`:" , ex )
            continue

        conf = rtde_config.ConfigFile("ur5_interface_configuration.xml")
        output_names , output_types = conf.get_recipe('out')

        if _ENABLE_LOG:
            log_print( "output_names:" , output_names )
            log_print( "output_types:" , output_types )

        try:
            rtde_proxy = rtde.RTDE(robot_arm_ip, 30004)
            rtde_proxy.connect()
        except Exception as ex:
            if _ENABLE_LOG:
                log_print( "Encountered a problem while starting `rtde.RTDE`:" , ex )
            continue

        if not rtde_proxy.send_output_setup(output_names, output_types):
            print('wrong config file')
            if _ENABLE_LOG:
                log_print( 'rtde_proxy.send_output_setup: wrong config file' )
                log_end()

            exit()
        # If we got to this point, all is well
        break
    except Exception as ex:
        if _ENABLE_LOG:
            log_print( "There was a problem establishing a connection:" , ex )
        continue
else:
    if _ENABLE_LOG:
        msg = "ALL ATTEMPTS TO CONNECT FAILED!"
        log_print( msg )
        log_end()
        raise RuntimeError( msg )
    
if _ENABLE_LOG:
   log_end()

def test_rtde_server():
    """ Return something descriptive """
    return "Server is live"
    
def get_tcp_pose():
    print( "About to get TCP pose ..." )
    rtde_proxy.send_start()
    state = rtde_proxy.receive()
    tcp_pose = state.actual_TCP_pose
    rtde_proxy.send_pause()
    print( "Got TCP pose!" )
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
    tcp_force = state.actual_TCP_force
    rtde_proxy.send_pause()
    return tcp_force

def get_robot_status():
    rtde_proxy.send_start()
    state = rtde_proxy.receive()
    rtde_proxy.send_pause()
    bits = state.robot_status_bits
    return bits

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
server.register_function(test_rtde_server,"test_rtde_server")

server.serve_forever()

