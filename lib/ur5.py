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
        subprocess.call("/home/nvidia/rmstudio/lib/rtde/run_ur5_rtde.sh")
        
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
        """ 
        Gives the tcp position of the effector.
        
        Return
        ------
        tcp_pose: [6,] list
            The pose of the effector.\n
            [x, y, z, rX, rY, rZ]
        
        """
        return self.proxy.get_tcp_pose()
    
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
    
    def get_camera_transform(self):
        """
        Gets the transformation matrix that describes the global position and rotation of the camera lense. \
        This transformation matrix is useful for transforming a local point cloud (relative to the camera) to \
        a global coordinate system (relative to the base of the robot) using transform(points).
        
        Return
        ------
        camera_transform: [4,4] ndarray
            Transformation matrix of the camera.\n
            [min_x,min_y,max_z]\n
            [max_x,min_y,max_z]\n
            [min_x,max_y,max_z]\n
            [max_x,max_y,max_z]
        
        """
        tcp_pose = self.get_tcp_pose()
        camera_pose  = self.pose_trans(tcp_pose, self.camera_offset)
        camera_transform = self.convert_pose_to_transform(camera_pose)
        return camera_transform
    
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
    
    def normalize_wrist_angle(self):
        """
        Retrieves the wrist joint angle and normalizes it to within 180deg of the zeros position. \
        This decreases the risk for over-rotation.
        """
        joint_angles = self.proxy.get_joint_angles()
        if joint_angles[5] > np.pi:
            joint_angles[5] -= 2*np.pi
        elif joint_angles[5] < -np.pi:
            joint_angles[5] += 2*np.pi
        self.set_joint_angles(joint_angles)
                
    
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
        Move the robot to the target pose in linear joint space. Note: if no speed or acceleration is set \
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
        
        if frame not in ["base","tool","base_lock_rot"]:
            raise NameError('Invalid reference frame')
        
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
        if frame=="tool":
            target = self.pose_trans(self.get_tcp_pose(),target)
        if frame=="base_lock_rot":
            target[:3] = list(np.add(self.get_tcp_pose()[:3],target[:3]))
        movement = ("movej(p{0}, a={1}, v={2})".format(target,accel,speed) + "\n").encode()
        self.move(movement,stop_condition)
        return 1
    
    def movel(self,target,speed=None,accel=None,speed_per=None,accel_per=None,frame="base",stop_condition='dummy'):
        """ 
        Move the robot in a linear tcp space to the target pose. Note: if no speed or acceleration is set \
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
        
        if frame not in ["base","tool","base_lock_rot"]:
            raise NameError('Invalid reference frame')
        
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
        if frame=="tool":
            target = self.pose_trans(self.get_tcp_pose(),target)
        if frame=="base_lock_rot":
            target[:3] = list(np.add(self.get_tcp_pose()[:3],target[:3]))
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
        
        if frame not in ["base","tool","base_lock_rot"]:
            raise NameError('Invalid reference frame')
        
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
        if frame=="tool":
            target = self.pose_trans(self.get_tcp_pose(),target)
        if frame=="base_lock_rot":
            target[:3] = list(np.add(self.get_tcp_pose()[:3],target[:3]))
        movement = ("movep(p{0}, a={1}, v={2}, r={3})".format(target,accel,speed,radius) + "\n").encode()
        self.move(movement,stop_condition)
        return 1
    
    def set_joint_angles(self,target,speed=None,accel=None,speed_per=None,accel_per=None,stop_condition='dummy'):
        """ 
        Move the robot in linear joint space to the target joint angles. Note: if no speed or acceleration is set \
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
        Move the robot in linear tool space to the target joint angles. Note: if no speed or acceleration is set \
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
    
    def spiral(self,angleToStep,startRadius,stepSize=.001,maxAngle=100000,maxRadius=100000,stop_condition=dummy_stop,speed=0.02,accel=1,frame="tool"):
        r = startRadius
        x = 0
        y = 0
        pos = [0,0,0,0,0,0]
        startPos = self.get_tcp_pose()
        posRotation = startPos[3:]
        phi = angleToStep
        count = 1
        allPoses = []
        while phi < maxAngle and r < maxRadius:
            phi=phi+angleToStep
            x=np.cos(np.deg2rad(phi))*r
            y=np.sin(np.deg2rad(phi))*r        
            pos[0]=x
            pos[1]=y
            r=r+stepSize

            if frame == "tcp":
                startPos[3:] = 3.14,0,0
                startPos = self.pose_trans(startPos,pos)
                startPos[3:] = posRotation 
            allPoses.append(self.pose_trans(startPos, pos))
        for posDex, pose in enumerate(allPoses):
            if stop_condition():
                break
            if(posDex + 1 < len(allPoses)):
                self.movep(pose, speed=speed, accel=accel, radius=1e-6, stop_condition=stop_condition)
            else:
                self.movep(pose, speed=speed, accel=accel, radius=0.00001, stop_condition=stop_condition)

    def align_gripper_with_axis(self,lock_roll=False,lock_pitch=False,lock_yaw=False):
        """
        Alignes the gripper with the nearest rotational axis (principle cartesian axes).
        
        Parameters
        ----------
        lock_roll: bool
            Should the current roll be locked?
        lock_pitch: bool
            Should the current pitch be locked?
        lock_yaw: bool
            Should the current yaw be locked?
        """
        transform = self.convert_pose_to_transform(self.get_tcp_pose())
        rot_matrix = transform[0:3, 0:3]
        R = self.rotation_matrix_to_RPY(rot_matrix)
        for i, value in enumerate(R):
            if(i==0 and lock_pitch): continue
            if(i==1 and lock_yaw): continue
            if(i==2 and lock_roll): continue
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
        rot_matrix = self.RPY_to_rotation_matrix(R)
        transform[0:3, 0:3] = rot_matrix
        pose = self.convert_transform_to_pose(transform)
        self.movej(pose)

    def rotate_wrist(self, z_rotation_deg):
        """
        Rotates the wrist joint angle by some number of degrees
        
        Parameters
        ----------
        z_rotation_deg: float
            Degree rotation of wrist, from current rotation. 
        """
        ja = self.get_joint_angles()
        self.set_joint_angles(np.add(ja,[0,0,0,0,0,math.radians(z_rotation_deg)]))
        
    def zero_wrist(self):
        """
        Sets the wrist joint angle to zero
        """
        ja = self.get_joint_angles()
        ja[5] = 0
        self.set_joint_angles(ja)
        
    def set_to_current_rotation(self, pose):
        """
        Sets the rotation of a tcp pose to that of the current alignment of the gripper tcp. 
        
        Parameters
        ----------
        pose: ([6,] list)
            Target tcp pose [x, y, z, rX, rY, rZ].
            
        Returns
        -------
        pose_aligned: ([6,] list)
            Target tcp pose [x, y, z, rX_tpc, rY_tpc, rZ_tpc].
        """
        pose[3:] = self.get_tcp_pose()[3:]
        
        return pose
        
        
