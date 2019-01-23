import serial
import time

class OpenCM:
    def __init__(self):
        #set constants
        self.MAX_WIDTH = 108 
        self.MAX_TORQUE = 200
        
        #init serial connection
        port = "/dev/opencm"
        baudrate = 9600
        timeout = 3
        self.ser = serial.Serial(port=port,baudrate=baudrate,timeout=timeout)
        self.ser.close()
        self.ser.open()

    #Gripper Setup Commands====================================================================================================

    def activate_gripper(self):
        self.ser.flushInput()
        self.ser.write('a\r'.encode())
        return 1 

    def deactivate_gripper(self):
        self.ser.flushInput()
        self.ser.write('d\r'.encode())
        return 1

    def zero_gripper(self):
        self.ser.flushInput()
        self.ser.write('z\r'.encode())
        time.sleep(15)
        return 1

    #Gripper Set Commands======================================================================================================

    def open_gripper(self):
        self.ser.flushInput()
        self.ser.write('o\r'.encode())
        return int(self.ser.readline().decode())

    def close_gripper(self):
        self.ser.flushInput()
        self.ser.write('c\r'.encode())
        return int(self.ser.readline().decode())


    def set_gripper_torque(self,torque):
        if  torque > self.MAX_TORQUE: 
            torque = self.MAX_TORQUE
        elif torque < 0:
            torque = 0
        self.ser.write('t{}\r'.format(torque).encode())
        return 1

    def set_gripper_torque_right(self,torque):
        if  torque > self.MAX_TORQUE: 
            torque = self.MAX_TORQUE
        elif torque < 0:
            torque = 0
        self.ser.flushInput()
        self.ser.write(('){}\r'.format(torque)).encode())
        return 1

    def set_gripper_torque_left(self,torque):
        if  torque > self.MAX_TORQUE: 
            torque = self.MAX_TORQUE
        elif torque < 0:
            torque = 0
        self.ser.flushInput()
        self.ser.write(('({}\r'.format(torque)).encode())
        return 1

    def set_gripper_width(self,width):
        width = int(float(width) * 1000.0)
        if width > self.MAX_WIDTH:
            width = self.MAX_WIDTH
        elif width < 0:
            width = 0
        self.ser.flushInput()
        self.ser.write(('w{}\r'.format(width)).encode())
        return int(self.ser.readline().decode()[0])

    def set_gripper_width_right(self,width):
        width = int(float(width) * 2000.0)
        if width > 2*self.MAX_WIDTH:
            width = 2*self.MAX_WIDTH
        elif width < 0:
            width = 0
        self.ser.write(('>{}\r'.format(width)).encode())
        return int(self.ser.readline().decode()[0])


    def set_gripper_width_left(self,width):
        width = int(float(width) * 2000.0)
        if width > 2*self.MAX_WIDTH:
            width = 2*self.MAX_WIDTH
        elif width < 0:
            width = 0
        self.ser.flushInput()
        self.ser.write(('<{}\r'.format(width)).encode())
        return int(self.ser.readline().decode()[0])

    def set_min_object_size(self,object_size):
        self.ser.flushInput()
        self.ser.write(('m{}\r'.format(object_size)).encode())
        return int(self.ser.readline().decode()[0])

    def set_red_led(self,color):
        color = 255-color
        self.ser.flushInput()
        self.ser.write(('r{}\r'.format(color)).encode())
        return 

    def set_green_led(self,color):
        color = 255-color
        self.ser.flushInput()
        self.ser.write(('g{}\r'.format(color)).encode())
        return 

    def set_blue_led(self,color):
        color = 255-color
        self.ser.flushInput()
        self.ser.write(('b{}\r'.format(color)).encode())
        return 

    def set_dance_mode(self):
        self.ser.flushInput()
        self.ser.write('%\r'.encode())
        return int(self.ser.readline().decode()[0])


    #Get Gripper Commands =====================================================================================================

    def get_software_version(self):
        self.ser.flushInput()
        self.ser.write('S\r'.encode())
        return int(self.ser.readline().decode())

    def get_gripper_width(self):
        for i in range(20):
            self.ser.flushInput()
            self.ser.write('W\r'.encode())
            try:
                width = int(self.ser.readline().decode()[:-2])
                width /= 1000
                break
            except:
                pass
        return width

    def get_motor_temperature(self):
        self.ser.flushInput()
        self.ser.write('T\r'.encode())
        return self.ser.readline().decode()[:-2]
