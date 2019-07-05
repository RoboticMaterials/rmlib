from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
import netifaces as ni

class XMLRPC_RMLib:  
    def run_xmlrpc(self):   
        rm_xmlrpc_port = 8100
        ip = ni.ifaddresses('eth0')[ni.AF_INET][0]['addr']
       
        class RequestHandler(SimpleXMLRPCRequestHandler):
            rpc_paths = ('/RPC2')

            def do_OPTIONS(self):
                self.send_response(200)
                self.end_headers()

            # Add these headers to all responses
            def end_headers(self):
                self.send_header("Access-Control-Allow-Headers", "Origin, X-Requested-With, Content-Type, Accept")
                self.send_header("Access-Control-Allow-Origin", "*")
                SimpleXMLRPCRequestHandler.end_headers(self)
        try:
            self.server = SimpleXMLRPCServer((ip, rm_xmlrpc_port), requestHandler=RequestHandler)
        except:
            pass
        
        self.server.register_introspection_functions()
        
        # General Functions
        self.server.register_function(self.open_gripper,'open_gripper')
        self.server.register_function(self.close_gripper,'close_gripper')
        self.server.register_function(self.set_gripper_torque,'set_gripper_torque')
        self.server.register_function(self.set_gripper_width,'set_gripper_width')
        self.server.register_function(self.align_gripper_with_axis,'align_gripper_with_axis')
        self.server.register_function(self.get_tcp_pose_vec,'get_tcp_pose_vec')
        self.server.register_function(self.movej,'movej') 
        
        
        # Feature Functions
        self.server.register_function(self.print_feature_lib,'print_feature_lib')
        self.server.register_function(self.save_feature_lib,'save_feature_lib')
        self.server.register_function(self.load_feature_lib,'load_feature_lib')
        self.server.register_function(self.update_point_cloud,'update_point_cloud')
        self.server.register_function(self.get_feature_capture,'get_feature_capture')
        self.server.register_function(self.wizard_segment_cloud,'wizard_segment_cloud')
        self.server.register_function(self.wizard_filter_clouds_size,'wizard_filter_clouds_size')
        self.server.register_function(self.wizard_remove_planar_surface,'wizard_remove_planar_surface')
        self.server.register_function(self.wizard_remove_container,'wizard_remove_container')
        self.server.register_function(self.init_auto_tune_nbscan,'init_auto_tune_nbscan')
        self.server.register_function(self.step_auto_tune_nbscan,'step_auto_tune_nbscan')
        self.server.register_function(self.get_auto_tune_nbscan_best,'get_auto_tune_nbscan_best')
        self.server.register_function(self.wizard_set_view_cloud,'wizard_set_view_cloud')
        
        # URCap Functions
        self.server.register_function(self.get_urcap_feature_list,'get_object_defs')
        self.server.register_function(self.get_urcap_feature_pose,'get_object_info')

        print("Starting RM Wizard Server on %s" % ip)
        
        # Clear cloud
        view_clear = self.PC_Viewer()
        view_clear.saveToFile(view='base')
        
        self.server.serve_forever()
