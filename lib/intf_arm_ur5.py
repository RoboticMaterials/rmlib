import subprocess
import socket
import netifaces as ni
import xmlrpc.client
import time
import numpy as np
import math

class UR5:
    def __init__(self):
        #run rtde server in pyhton 2
        subprocess.call("/home/nvidia/rmstudio/lib/ur_servers/run_ur5_rtde.sh")
        
        #connect to rtde server as xmlrpc client
        ip = ni.ifaddresses('eth0')[ni.AF_INET][0]['addr']
        self.proxy = xmlrpc.client.ServerProxy("http://{}:8001/RPC2".format(ip))
        
        #start socket to ur5
        self.s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.s.connect((self.robot_arm_ip, 30003))
        time.sleep(0.05)
    
    def dummy_stop(self):
        return False 
    
    def get_tcp_pose_vec(self):
        """ 
        Gives the tcp position of the effector.
        
        Return
        ------
        tcp_pose: [6,] list
            The pose of the effector.\n
            [x, y, z, rX, rY, rZ]
        
        """
        return self.proxy.get_tcp_pose()
    
    def get_tcp_pose(self):
        """ 
        Gives the tcp position of the effector.
        
        Return
        ------
        tcp_pose: [6,] list
            The pose of the effector.\n
            [x, y, z, rX, rY, rZ]
        
        """
        return self.pose_vec_to_mtrx(self.get_tcp_pose_vec())
    
    def get_tcp_force(self):
        """
        Gets the 6-axis force magnitudes on the tcp.
        
        Return
        ------
        tcp_force: [6,] list
            The force on the effector.\n
            [nx, ny, nz, rx, ry, rz]
        
        """
        return self.proxy.get_tcp_force()
    
    def get_joint_angles(self):
        """ 
        Get the configuration of the robot in joint space.
        
        Return
        ------
        joint angles: [6,] list
            The radian angles of each joint. \n
            [base, shoulder, elbow, wrist_1, wrist_2, wrist_3]
        
        """
        return self.proxy.get_joint_angles()  
    
    def send_command_to_robot(self,command):
        self.s.send(command)
        time.sleep(0.1)

    def wait_until_robot_is_finished(self,stop_condition='dummy'):
        if stop_condition == 'dummy':
            stop_condition = self.dummy_stop
        while self.proxy.get_robot_status() == 3:
            if stop_condition():
                self.movel(self.proxy.get_tcp_pose())
                break
                
    def move(self,movement,stop_condition='dummy'):
        self.send_command_to_robot(movement)
        self.wait_until_robot_is_finished(stop_condition=stop_condition)
           
    def movej(self,target,speed=None,accel=None,speed_per=None,accel_per=None,frame="base",stop_condition='dummy'):
        """ 
        Move the arm to the target pose in linear joint space. Note: if no speed or acceleration is set \
        The robot will move at the default speed and acceleration, set in rm_config.
        
        Parameters
        ----------
        target: [6,] list
            Target tcp pose.\n
            [x, y, z, rX, rY, rZ]
        speed: float (optional)
            Joint velocity (rad/s).
        acceleration: float (optional)
            Joint acceleration (rad/s^2).
        speed_per: float (optional)
            Joint velocity percent, 0 to 1: 1 being maximum allowed joint speed. 
        acceleration_per: float (optional)
            Joint acceleration percent, 0 to 1: 1 being maximum allowed joint acceleration. 
        frame: ["base","tool","base_lock_rot"] (optional)
            Frame of movement.\n
            base: Relative to the robot base.\n
            tool: Realative to the frame of reference of the gripper.
            base_lock_rot: Translate only relative to the robot base. 
        stop_condition: function (optional)
            The condition that would cause the movement to hault. \n
            Example: stop_condition = lambda: rm.get_tcp_force()[2] < -4
            
        Return
        ------
        1
        """
        
        #Select speed value
        if speed_per is not None:
            speed = speed_per*self.arm_max_joint_speed
        elif speed is not None:
            pass
        else:
            speed = self.arm_default_joint_speed     
        #Select acceleration value    
        if accel_per is not None:
            accel = accel_per*self.arm_max_joint_accel
        elif accel is not None:
            pass
        else:
            accel = self.arm_default_joint_accel
        #Limit speed to max value
        if speed > self.arm_max_joint_speed:
            speed = self.arm_max_joint_speed
        #Limit acceleration to max value
        if accel > self.arm_max_joint_accel:
            accel = self.arm_max_joint_accel
    
        if frame=="tool":
            if(type(target)==list and len(target) == 6):
                target = self.pose_vec_to_mtrx(target)
            target = self.get_tcp_pose().dot(target)
        
        if type(target) == np.ndarray and target.shape == (4,4): 
            target = self.pose_mtrx_to_vec(target)
            
        movement = ("movej(p{0}, a={1}, v={2})".format(target,accel,speed) + "\n").encode()
        self.move(movement,stop_condition)
        return 1
    
    def movel(self,target,speed=None,accel=None,speed_per=None,accel_per=None,frame="base",stop_condition='dummy'):
        """ 
        Move the arm in a linear tcp space to the target pose. Note: if no speed or acceleration is set \
        The robot will move at the default speed and acceleration, set in rm_config.
        
        Parameters
        ----------
        target: [6,] list
            Target tcp pose.\n
            [x, y, z, rX, rY, rZ]
        speed: float (optional)
            linear velocity (m/s).
        acceleration: float (optional)
            linear acceleration (m/s^2).
        speed_per: float (optional)
            linear velocity percent, 0 to 1: 1 being maximum allowed linear speed. 
        acceleration_per: float (optional)
            linear acceleration percent, 0 to 1: 1 being maximum allowed linear acceleration. 
        frame: ["base","tool","base_lock_rot"] (optional)
            Frame of movement.\n
            base: Relative to the robot base.\n
            tool: Realative to the frame of reference of the gripper.
            base_lock_rot: Translate only relative to the robot base. 
        stop_condition: function (optional)
            The condition that would cause the movement to hault. \n
            Example: stop_condition = lambda: rm.get_tcp_force()[2] < -4
            
        Return
        ------
        1
        """
        
        #Select speed value
        if speed_per is not None:
            speed = speed_per*self.arm_max_linear_speed
        elif speed is not None:
            pass
        else:
            speed = self.arm_default_linear_speed     
        #Select acceleration value    
        if accel_per is not None:
            accel = accel_per*self.arm_max_linear_accel
        elif accel is not None:
            pass
        else:
            accel = self.arm_default_linear_accel
        #Limit speed to max value
        speed = min(speed,self.arm_max_linear_speed)
        #Limit acceleration to max value
        accel = min(accel,self.arm_max_linear_accel)

        if frame=="tool":
            target = self.get_tcp_pose().dot(target)
        
        target = self.pose_mtrx_to_vec(target)
        movement = ("movel(p{0}, a={1}, v={2})".format(target,accel,speed) + "\n").encode()
        self.move(movement,stop_condition)
        return 1
    
    def movep(self,target,speed=None,accel=None,speed_per=None,accel_per=None,radius=0.0,frame="base",stop_condition=dummy_stop):
        """ Move the tool linearly with constant speed with circular blends. Note: if no speed or acceleration is set \
        The robot will move at the default speed and acceleration, set in rm_config.
        
        Parameters
        ----------
        target: [6,] list
            Target tcp pose.\n
            [x, y, z, rX, rY, rZ]
        speed: float (optional)
            Joint velocity (rad/s).
        acceleration: float (optional)
            Joint acceleration (rad/s^2).
        speed_per: float (optional)
            Joint velocity percent, 0 to 1: 1 being maximum allowed joint speed. 
        acceleration_per: float (optional)
            Joint acceleration percent, 0 to 1: 1 being maximum allowed joint acceleration. 
        frame: ["base","tool","base_lock_rot"] (optional)
            Frame of movement.\n
            base: Relative to the robot base.\n
            tool: Realative to the frame of reference of the gripper.
            base_lock_rot: Translate only relative to the robot base. 
        stop_condition: function (optional)
            The condition that would cause the movement to hault. \n
            Example: stop_condition = lambda: rm.get_tcp_force()[2] < -4
            
        Return
        ------
        1
        """
        
        #Select speed value
        if speed_per is not None:
            speed = speed_per
        elif speed is not None:
            pass
        else:
            speed = self.arm_default_linear_speed     
        #Select acceleration value    
        if accel_per is not None:
            accel = accel_per
        elif accel is not None:
            pass
        else:
            accel = self.arm_default_linear_accel
        #Limit speed to max value
        if speed > self.arm_max_linear_speed:
            speed = self.arm_max_linear_speed
        #Limit acceleration to max value
        if accel > self.arm_max_linear_accel:
            accel = self.arm_max_linear_accel

        if frame=="tool":
            target = self.get_tcp_pose().dot(target)
            
        target = self.pose_mtrx_to_vec(target)
        movement = ("movep(p{0}, a={1}, v={2}, r={3})".format(target,accel,speed,radius) + "\n").encode()
        self.move(movement,stop_condition)
        return 1
    
    def set_joint_angles(self,target,speed=None,accel=None,speed_per=None,accel_per=None,stop_condition='dummy'):
        """ 
        Move the arm in linear joint space to the target joint angles. Note: if no speed or acceleration is set \
        The robot will move at the default speed and acceleration, set in rm_config.
        
        Parameters
        ----------
        target: [6,] list
            Target joint angles.\n
            [base, shoulder, elbow, wrist_1, wrist_2, wrist_3]
        speed: float (optional)
            Joint velocity (rad/s).
        acceleration: float (optional)
            Joint acceleration (rad/s^2).
        speed_per: float (optional)
            Joint velocity percent, 0 to 1: 1 being maximum allowed joint speed. 
        acceleration_per: float (optional)
            Joint acceleration percent, 0 to 1: 1 being maximum allowed joint acceleration. 
        stop_condition: function (optional)
            The condition that would cause the movement to hault. \n
            Example: stop_condition = lambda: rm.get_tcp_force()[2] < -4
            
        Return
        ------
        1
        """
        
        #Select speed value
        if speed_per is not None:
            speed = speed_per
        elif speed is not None:
            pass
        else:
            speed = self.arm_default_joint_speed     
        #Select acceleration value    
        if accel_per is not None:
            accel = accel_per
        elif accel is not None:
            pass
        else:
            accel = self.arm_default_joint_accel
        #Limit speed to max value
        if speed > self.arm_max_joint_speed:
            speed = self.arm_max_joint_speed
        #Limit acceleration to max value
        if accel > self.arm_max_joint_accel:
            accel = self.arm_max_joint_accel
        
        if isinstance(target, np.ndarray):
            target = target.tolist()
        movement = ("movej({0}, a={1}, v={2})".format(target,accel,speed) + "\n").encode()
        self.move(movement,stop_condition)
        return 1
    
    def set_joint_angles_l(self,target,speed=None,accel=None,speed_per=None,accel_per=None,stop_condition='dummy'):
        """ 
        Move the arm in linear tool space to the target joint angles. Note: if no speed or acceleration is set \
        The robot will move at the default speed and acceleration, set in rm_config.
        
        Parameters
        ----------
        target: [6,] list
            Target joint angles.\n
            [base, shoulder, elbow, wrist_1, wrist_2, wrist_3]
        speed: float (optional)
            linear velocity (m/s).
        acceleration: float (optional)
            linear acceleration (m/s^2).
        speed_per: float (optional)
            linear velocity percent, 0 to 1: 1 being maximum allowed linear speed. 
        acceleration_per: float (optional)
            linear acceleration percent, 0 to 1: 1 being maximum allowed linear acceleration. 
        stop_condition: function (optional)
            The condition that would cause the movement to hault. \n
            Example: stop_condition = lambda: rm.get_tcp_force()[2] < -4
            
        Return
        ------
        1
        """
        
        #Select speed value
        if speed_per is not None:
            speed = speed_per
        elif speed is not None:
            pass
        else:
            speed = self.arm_default_linear_speed     
        #Select acceleration value    
        if accel_per is not None:
            accel = accel_per
        elif accel is not None:
            pass
        else:
            accel = self.arm_default_linear_accel
        #Limit speed to max value
        if speed > self.arm_max_linear_speed:
            speed = self.arm_max_linear_speed
        #Limit acceleration to max value
        if accel > self.arm_max_linear_accel:
            accel = self.arm_max_linear_accel
        
        if isinstance(target, np.ndarray):
            target = target.tolist()
        movement = ("movel({0}, a={1}, v={2})".format(target,real_accel,real_speed) + "\n").encode()
        self.move(movement,stop_condition)
        return 1
    
    

        
        
