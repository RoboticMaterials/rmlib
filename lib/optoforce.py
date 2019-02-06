import socket
import time
import struct
import json

class OptoForce:

    def __init__(self):
        self.rti = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.ci = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.open_force_sensor()
        self.init_force_sensor()

    def get_wrist_force(self):
        message = bytes(2) + b'\x02\x02' + bytes(72) 
        self.rti.send(message)
        data = self.rti.recv(58)
        return list(struct.unpack('!6f',data[-24:]))

    def bias_wrist_force(self): 
        msg = dict()
        msg["message_id"] = ""
        msg["command"] = dict()
        msg["command"]["id"] = "bias"

        msg = (json.dumps(msg) + "\r\n\r\n").encode()
        self.ci.send(msg)
        data = self.ci.recv(1024)
        self.get_wrist_force()
        return data

    def close_force_sensor(self):
        rti.close()
        ci.close()

    def open_force_sensor(self):
        self.rti.connect(("{}".format(self.force_sensor_ip),32000))
        self.ci.connect(("{}".format(self.force_sensor_ip),32002))
        time.sleep(0.1)

    def init_force_sensor(self):
        msg = dict()
        msg["message_id"] = ""
        msg["command"] = dict()
        msg["command"]["id"] = "configuration"
        msg["command"]["robot_cycle"] = 1
        msg["command"]["sensor_cycle"] = 4
        msg["command"]["max_translational_speed"] = 1.0
        msg["command"]["max_rotational_speed"] = 1.0
        msg["command"]["max_translational_acceleration"] = 1.0
        msg["command"]["max_rotational_acceleration"] = 1.0

        msg = (json.dumps(msg) + "\r\n\r\n").encode()
        self.ci.send(msg)
        data = self.ci.recv(1024)
        return 
    

