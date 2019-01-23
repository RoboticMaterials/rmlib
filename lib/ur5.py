import subprocess
import socket
import netifaces as ni
import xmlrpc.client
import time
import numpy as np

class UR5:
    def __init__(self):
        #run rtde server in pyhton 2
        subprocess.call("/home/nvidia/rmstudio/lib/run_ur5_rtde.sh")
        #connect to rtde server as xmlrpc client
        
        ip = ni.ifaddresses('eth0')[ni.AF_INET][0]['addr']
        self.proxy = xmlrpc.client.ServerProxy("http://{}:8001/RPC2".format(ip))
        
        #start socket to ur5
        self.s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.s.connect((self.robot_arm_ip, 30003))
        time.sleep(0.05)
    
    def dummy_stop(self):
        return False 
    
    def get_tcp_pose(self):
        """ return the pose of the effector [ x , y , z , rX , rY , rZ ] """
        tcp_pose = self.proxy.get_tcp_pose()
        return tcp_pose
    
    def get_camera_transform(self):
        tcp_pose = self.get_tcp_pose()
        camera_pose  = self.pose_trans( tcp_pose , self.camera_offset )
        camera_transform = self.convert_pose_to_transform( camera_pose )
        return camera_transform
    
    def get_joint_angles(self):
        """ return the configuration of the robot in joint space """
        return self.proxy.get_joint_angles()  
    
    def normalize_wrist_angle(self,joint_angles):
        joint_angles = self.proxy.get_joint_angles()
        if joint_angles[5] > np.pi:
            joint_angles[5] -= 2*np.pi
        elif joint_angles[5] < -np.pi:
            joint_angles[5] += 2*np.pi
        return joint_angles
                
    
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
           
    def movej(self,target,speed=1.05,acceleration=1.4,frame="tcp",stop_condition='dummy'):
        """ move the robot to the goal pose """
        if isinstance(target, np.ndarray):
            target = target.tolist()
        if frame=="tool":
            target = self.pose_trans(self.get_tcp_pose(),target)
        movement = ("movej(p{0}, a={1}, v={2})".format(target,acceleration,speed) + "\n").encode()
        self.move(movement,stop_condition)
        return 1
    
    def movel(self,target,speed=0.25,acceleration=1.2,frame="tcp",stop_condition='dummy'):
        """ move the robot in a linear task space path to the goal pose """
        if isinstance(target, np.ndarray):
            target = target.tolist()
        if frame=="tool":
            target = self.pose_trans(self.get_tcp_pose(),target)
        movement = ("movel(p{0}, a={1}, v={2})".format(target,acceleration,speed) + "\n").encode()
        self.move(movement,stop_condition)
        return 1
    
    def set_joint_angles(self,target,speed=1.05,acceleration=1.4,stop_condition='dummy'):
        """ move the robot in a joint space path to the goal joint angles """
        if isinstance(target, np.ndarray):
            target = target.tolist()
        movement = ("movej({0}, a={1}, v={2})".format(target,acceleration,speed) + "\n").encode()
        self.move(movement,stop_condition)
        return 1

    def align_gripper_with_axis(self):
        transform = self.convert_pose_to_transform(self.get_tcp_pose())
        rot_matrix = transform[0:3, 0:3]
        R = self.rotation_matrix_to_RPY(rot_matrix)
        j = 0
        for i, value in enumerate(R):
            if value > -3.142 and value <= -2.36:
                R[i] = -3.14 #-180
            elif value > -2.36 and value <= -0.79:
                R[i] = -1.57 #-90
            elif value > -0.79 and value <= 0.79:
                R[i] = 0 #0
            elif value > 0.79 and value <= 2.36:
                R[i] = 1.57 #90
            elif value > 2.36 and value <= 3.142:
                R[i] = 3.14 #180
            else:
                raise NameError('ERROR -Austin')
            j =+ 1
        rot_matrix = self.RPY_to_rotation_matrix(R)
        transform[0:3, 0:3] = rot_matrix
        pose = self.convert_transform_to_pose(transform)
        self.movej(pose) 
    
    
    def align_gripper_with_z(self):
        transform = self.convert_pose_to_transform(self.get_tcp_pose())
        rot_matrix = transform[0:3, 0:3]
        R = self.rotation_matrix_to_RPY(rot_matrix)

        for i in range(2):
            if R[i] > -3.142 and R[i] <= -2.36:
                R[i] = -3.14 #-180
            elif R[i] > -2.36 and R[i] <= -0.79:
                R[i] = -1.57 #-90
            elif R[i] > -0.79 and R[i] <= 0.79:
                R[i] = 0 #0
            elif R[i] > 0.79 and R[i] <= 2.36:
                R[i] = 1.57 #90
            elif R[i] > 2.36 and R[i] <= 3.142:
                R[i] = 3.14 #180
            else:
                raise NameError('ERROR -Davis')

        rot_matrix = self.RPY_to_rotation_matrix(R)
        transform[0:3, 0:3] = rot_matrix
        pose = self.convert_transform_to_pose(transform)
        self.movej(pose) 
