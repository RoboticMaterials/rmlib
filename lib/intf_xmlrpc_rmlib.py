from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
import netifaces as ni
from rm_config import rm_config


class XMLRPC_RMLib:
    def run_xmlrpc(self):
        rm_xmlrpc_port = 8100
        self.is_dashboard_dead = False

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
        try:
            self.server = SimpleXMLRPCServer(
                ("", rm_xmlrpc_port), requestHandler=RequestHandler, logRequests=True)
        except:
            pass

        self.server.register_introspection_functions()

        # General Functions
        self.server.register_function(self.open_gripper, 'open_gripper')
        self.server.register_function(self.close_gripper, 'close_gripper')
        self.server.register_function(
            self.set_gripper_torque, 'set_gripper_torque')
        self.server.register_function(
            self.set_gripper_width, 'set_gripper_width')
        self.server.register_function(
            self.get_gripper_width, 'get_gripper_width')
        self.server.register_function(
            self.set_gripper_width_left, 'set_gripper_width_left')
        self.server.register_function(
            self.set_gripper_width_right, 'set_gripper_width_right')
        self.server.register_function(
            self.align_gripper_with_axis, 'align_gripper_with_axis')
        self.server.register_function(
            self.get_tcp_pose_vec, 'get_tcp_pose_vec')
        self.server.register_function(self.movej, 'movej')
        self.server.register_function(self.kill_xmlrpc_server, 'kill_server')

        # Feature Functions
        self.server.register_function(
            self.print_feature_lib, 'print_feature_lib')
        self.server.register_function(
            self.save_feature_lib, 'save_feature_lib')
        self.server.register_function(
            self.load_feature_lib, 'load_feature_lib')

        # URCap Functions
        self.server.register_function(
            self.get_urcap_feature_list, 'get_object_defs')
        self.server.register_function(
            self.get_urcap_feature_pose, 'get_object_info')

        # Mobile Platform Functions
        if rm_config['import']['mobile_platform']:
            self.server.register_function(self.move_mp_to_waypoint, 'movecart')
        self.server.register_function(self.urcap_get_mp_waypoints, 'request_waypoints')
        self.server.register_function(self.urcap_return_mp_waypoints, 'get_waypoints')


        print("Starting RM XMLRPC Server")

        # Clear cloud
        view_clear = self.PC_Viewer()
        view_clear.save_to_file(view='base')

        while not self.is_dashboard_dead:
            self.server.handle_request()

        return True

    def kill_xmlrpc_server(self):
        self.is_dashboard_dead = True
        return True
