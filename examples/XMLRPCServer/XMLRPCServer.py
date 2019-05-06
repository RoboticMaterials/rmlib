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
    try:
        rm
    except:
        import rmlib
        global rm
        rm = rmlib.RMLib()
#    import rmlib
#    global rm
#    rm = rmlib.RMLib()
    return True

def irimage():
	import cv2
	import numpy as np
	rm.set_laser_state(False)
	rm.set_auto_exposure_on(True)
	#rm.set_exposure(165000)
	
	img = rm.get_ir_image()
	img = cv2.flip(img,0)
	img = cv2.flip(img,1)
	img = np.array(img*255./img.max()).astype(np.uint8)

	cv2.imwrite('/media/ramdisk/snapshot.png',img)
	return True	

def get_object_defs():
        #...
        #return Features ("M3 Screw","Banana")

print("Starting XML-RPC server on port 8101")
server = SimpleXMLRPCServer(("", 8101),requestHandler=RequestHandler)
server.register_introspection_functions()
server.register_function(run_cmd)
server.register_function(init)
server.register_function(irimage)
server.serve_forever()
