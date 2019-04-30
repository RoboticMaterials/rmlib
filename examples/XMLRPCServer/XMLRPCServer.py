import sys
sys.path.append('/home/nvidia/dev_rmstudio/lib') 

from xmlrpc.server import SimpleXMLRPCServer
import xmlrpc.client
from xmlrpc.server import SimpleXMLRPCRequestHandler
import time
import base64
import netifaces as ni
import numpy as np
import sys
import traceback

#import rmlib
#rm = rmlib.RMLib()

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
    import rmlib
    global rm
    rm = rmlib.RMLib()
    return True

	

print("Starting XML-RPC server on port 8101")
server = SimpleXMLRPCServer(("", 8101),requestHandler=RequestHandler)
server.register_introspection_functions()
server.register_function(run_cmd)
server.register_function(init)
server.serve_forever()
