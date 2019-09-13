import socket
import time
import struct
import json


class OnRobot:

    def __init__(self):
        try:
            self.rti = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.ci = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.rti.settimeout(1)
            self.ci.settimeout(1)
            self.open_force_sensor()

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
            self.get_wrist_force()
        except:
            raise Exception('Error Connecting to Force Sensor')

    def revive_FT_client_socket(self):
        """ Test for a response an restart """
        hasErr = True
        loopLm = 10
        i = 0
        while hasErr and (i < loopLm):
            try:
                self.get_wrist_force()
                hasErr = False
            except Exception as err:
                print("There was some socket error:", err)
                hasErr = True
            if hasErr:
                print("Attempt", i+1)
                try:
                    self.__init__()
                except Exception as err:
                    print("While attempting to reconnect, encountered:", err)
            sleep(1)
            print("Socket INET:", socket.AF_INET, '\n')
            i += 1
        if not hasErr:
            print("\nSUCCESS in reviving FT socket")
        else:
            print("\nFAILURE in reviving FT socket")

    def get_wrist_force(self):
        """
        Retrieves the force and torque vector from the OptoForce.

        Returns
        -------
        force_torque_vector: [6,] list
            The first 3 elements desribe the force along each axis, and the second \
            3 describe the torque about each axis.\n
            [Fx, Fy, Fz, Tx, Ty, Tz]
        """

        message = bytes(2) + b'\x02\x02' + bytes(72)
        self.rti.send(message)
        data = self.rti.recv(58)
        return list(struct.unpack('!6f', data[-24:]))

    def bias_wrist_force(self):
        """
        Bias the wrist forces. Call this before expecting a force, so the \
        force sensor is properly calibrated for the gripper's current orientation. 
        """

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
        self.rti.connect(("{}".format(self.force_sensor_ip), 32000))
        self.ci.connect(("{}".format(self.force_sensor_ip), 32002))
        time.sleep(0.1)
