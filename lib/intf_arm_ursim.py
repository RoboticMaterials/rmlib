import subprocess
import socket
import netifaces as ni
import xmlrpc.client
import time
import numpy as np
import math


class URSIM:
    def __init__(self):
        self.tcp = np.subtract(np.divide(np.random.rand(6), 5), 0.1)
        self.jas = np.subtract(np.multiply(np.random.rand(6), 2*np.pi), np.pi)
        return True

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
        self.tcp += np.subtract(np.random.rand(6), 0.5)
        return self.tcp

    def get_tcp_pose(self):
        """ 
        Gives the tcp position of the effector.

        Return
        ------
        tcp_pose: [6,] list
            The pose of the effector.\n
            [x, y, z, rX, rY, rZ]

        """
        self.tcp += np.subtract(np.random.rand(6), 0.5)
        return self.pose_vec_to_mtrx(self.tcp)

    def get_tcp_force(self):
        """
        Gets the 6-axis force magnitudes on the tcp.

        Return
        ------
        tcp_force: [6,] list
            The force on the effector.\n
            [nx, ny, nz, rx, ry, rz]

        """
        return [0, 0, 0, 0, 0, 0]

    def get_joint_angles(self):
        """ 
        Get the configuration of the robot in joint space.

        Return
        ------
        joint angles: [6,] list
            The radian angles of each joint. \n
            [base, shoulder, elbow, wrist_1, wrist_2, wrist_3]

        """
        self.jas += np.divide(np.subtract(np.random.rand(6), 0.5), 20)
        return self.jas

    def movej(self, target, speed=None, accel=None, speed_per=None, accel_per=None, frame="base", stop_condition='dummy'):
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
        return True

    def movel(self, target, speed=None, accel=None, speed_per=None, accel_per=None, frame="base", stop_condition='dummy'):
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

        return True

    def movep(self, target, speed=None, accel=None, speed_per=None, accel_per=None, radius=0.0, frame="base", stop_condition=dummy_stop):
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

        return True

    def set_joint_angles(self, target, speed=None, accel=None, speed_per=None, accel_per=None, stop_condition='dummy'):
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
        return True

    def set_joint_angles_l(self, target, speed=None, accel=None, speed_per=None, accel_per=None, stop_condition='dummy'):
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

        return True
