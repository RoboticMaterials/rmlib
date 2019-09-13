import numpy as np
import math


class Movements:
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

    def get_base_to_camera_pose(self, cam='pc'):
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
        if cam == 'pc':
            camera_pose = tcp_pose.dot(self.tcp_to_pc_cam_pose)
        elif cam == 'ci':
            camera_pose = tcp_pose.dot(self.tcp_to_ci_cam_pose)
        else:
            raise Exception('cam: ' + cam + ' not supported')
        return camera_pose

    def dummy_stop(self):
        return False

    def spiral(self, angleToStep, startRadius, stepSize=.001, maxAngle=100000, maxRadius=100000, stop_condition=dummy_stop, speed=0.02, accel=1, frame="tool"):
        """ Move the gripper in a spiral until either the stop condtion is met or the spiral has been completed """
        r = startRadius
        x = 0
        y = 0
        startPos = self.get_tcp_pose()
        posRotation = startPos[3:]
        phi = angleToStep
        count = 1
        allPoses = []
        while phi < maxAngle and r < maxRadius:
            phi = phi + angleToStep
            x = np.cos(np.deg2rad(phi)) * r
            y = np.sin(np.deg2rad(phi)) * r
            r = r + stepSize
            allPoses.append(self.translate_pose(startPos, x, y))  # BASE FRAME
        for posDex, pose in enumerate(allPoses):
            if stop_condition():
                break
            if(posDex + 1 < len(allPoses)):
                self.movep(pose, speed=speed, accel=accel,
                           radius=1e-6, stop_condition=stop_condition)
            else:
                self.movep(pose, speed=speed, accel=accel,
                           radius=0.00001, stop_condition=stop_condition)

    def align_gripper_with_axis(self, lock_roll=False, lock_pitch=False, lock_yaw=False):
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
        pose = self.get_tcp_pose()
        rot_matrix = pose[0:3, 0:3]
        R = self.rotation_mtrx_to_rpy(rot_matrix)
        for i, value in enumerate(R):
            if(i == 0 and lock_pitch):
                continue
            if(i == 1 and lock_yaw):
                continue
            if(i == 2 and lock_roll):
                continue
            if value > -3.142 and value <= -2.36:
                R[i] = -3.14  # -180
            elif value > -2.36 and value <= -0.79:
                R[i] = -1.57  # -90
            elif value > -0.79 and value <= 0.79:
                R[i] = 0  # 0
            elif value > 0.79 and value <= 2.36:
                R[i] = 1.57  # 90
            elif value > 2.36 and value <= 3.142:
                R[i] = 3.14  # 180
            else:
                raise NameError('ERROR -Austin')
        rot_matrix = self.rpy_to_rotation_mtrx(R)
        pose[0:3, 0:3] = rot_matrix
        rtnVal = self.movej(pose)
        return True

    def rotate_joint(self, z_rotation_deg, joint=5):
        """
        Rotates the joint angle by some number of degrees

        Parameters
        ----------
        z_rotation_deg: float
            Degree rotation of wrist, from current rotation. 
        """
        ja = self.get_joint_angles()
        if joint == 5:
            self.set_joint_angles(
                np.add(ja, [0, 0, 0, 0, 0, math.radians(z_rotation_deg)]))
        if joint == 4:
            self.set_joint_angles(
                np.add(ja, [0, 0, 0, 0, math.radians(z_rotation_deg), 0]))
        if joint == 3:
            self.set_joint_angles(
                np.add(ja, [0, 0, 0, math.radians(z_rotation_deg), 0, 0]))
        if joint == 2:
            self.set_joint_angles(
                np.add(ja, [0, 0, math.radians(z_rotation_deg), 0, 0, 0]))
        if joint == 1:
            self.set_joint_angles(
                np.add(ja, [0, math.radians(z_rotation_deg), 0, 0, 0, 0]))
        if joint == 0:
            self.set_joint_angles(
                np.add(ja, [math.radians(z_rotation_deg), 0, 0, 0, 0, 0]))

    def zero_joint(self, joint=5):
        """
        Sets the wrist joint angle to zero
        """
        ja = self.get_joint_angles()
        ja[joint] = 0
        self.set_joint_angles(ja)

    def move_through_mtrx_of_poses(self, mtrx_of_poses, function, method='123', path='sequential', speed_per=None, accel_per=None):
        shape = mtrx_of_poses.shape
        if path == 'sequential':
            for i in range(shape[0]):
                for j in range(shape[1]):
                    for k in range(shape[2]):

                        # self.movej(mtrx_of_poses[i,j,k,:], speed_per=speed_per, accel_per = accel_per)
                        if method == "123":
                            try:
                                self.movej(
                                    mtrx_of_poses[i, j, k, :], speed_per=speed_per, accel_per=accel_per)
                            except Exception:
                                print('Failed to move')
                        if method == "132":
                            self.movej(
                                mtrx_of_poses[i, k, j, :], speed_per=speed_per, accel_per=accel_per)
                        if method == "213":
                            self.movej(
                                mtrx_of_poses[j, i, k, :], speed_per=speed_per, accel_per=accel_per)
                        if method == "231":
                            self.movej(
                                mtrx_of_poses[j, k, i, :], speed_per=speed_per, accel_per=accel_per)
                        if method == "312":
                            self.movej(
                                mtrx_of_poses[k, i, j, :], speed_per=speed_per, accel_per=accel_per)
                        if method == "321":
                            self.movej(
                                mtrx_of_poses[k, j, i, :], speed_per=speed_per, accel_per=accel_per)
                        function()

        if path == 'optimal':
            j_rev = False
            k_rev = False
            for i in range(shape[0]):
                if not j_rev:
                    j_range = range(shape[1])
                else:
                    j_range = reversed(range(shape[1]))
                for j in j_range:
                    if not k_rev:
                        k_range = range(shape[2])
                    else:
                        k_range = reversed(range(shape[2]))
                    for k in k_range:
                        if method == "123":
                            self.movej(
                                mtrx_of_poses[i, j, k, :], speed_per=speed_per, accel_per=accel_per)
                        if method == "132":
                            self.movej(
                                mtrx_of_poses[i, k, j, :], speed_per=speed_per, accel_per=accel_per)
                        if method == "213":
                            self.movej(
                                mtrx_of_poses[j, i, k, :], speed_per=speed_per, accel_per=accel_per)
                        if method == "231":
                            self.movej(
                                mtrx_of_poses[j, k, i, :], speed_per=speed_per, accel_per=accel_per)
                        if method == "312":
                            self.movej(
                                mtrx_of_poses[k, i, j, :], speed_per=speed_per, accel_per=accel_per)
                        if method == "321":
                            self.movej(
                                mtrx_of_poses[k, j, i, :], speed_per=speed_per, accel_per=accel_per)
                        function()
                    k_rev = not k_rev
                j_rev = not j_rev
        return

    def move_camera_over_tcp(self, cam='color'):
        if cam == 'color':
            pose = self.translate_pose(self.get_tcp_pose(
            ), x=-self.tcp_to_ci_cam_pose[0, 3], y=-self.tcp_to_ci_cam_pose[1, 3], frame='self')
        elif cam == 'cloud':
            pose = self.translate_pose(self.get_tcp_pose(
            ), x=-self.tcp_to_pc_cam_pose[0, 3], y=-self.tcp_to_pc_cam_pose[1, 3], frame='self')
        else:
            raise Exception('Camera '+str(cam)+' not supported')
        self.movel(pose, speed=0.1)

    def move_tcp_under_camera(self, cam='color'):
        if cam == 'color':
            pose = self.translate_pose(self.get_tcp_pose(
            ), x=self.tcp_to_ci_cam_pose[0, 3], y=self.tcp_to_ci_cam_pose[1, 3], frame='self')
        elif cam == 'cloud':
            pose = self.translate_pose(self.get_tcp_pose(
            ), x=self.tcp_to_pc_cam_pose[0, 3], y=self.tcp_to_pc_cam_pose[1, 3], frame='self')
        else:
            raise Exception('Camera '+str(cam)+' not supported')
        self.movel(pose, speed=0.1)

    def move_tcp_over_pose(self, pose, distance, rotate=False):
        if rotate == False:
            pose = self.combine_poses(
                rot_pose=self.get_tcp_pose(), trans_pose=pose)
        else:
            pose = self.align_pose_to_tcp(pose, frame='base')

        pose = self.translate_pose(pose, z=-distance, frame='self')
        self.movel(pose)

    def move_camera_over_pose(self, pose, distance, rotate=False, cam='color'):
        if rotate == False:
            pose = self.combine_poses(
                rot_pose=self.get_tcp_pose(), trans_pose=pose)
        else:
            pose = self.align_pose_to_tcp(pose, frame='base')

        if cam == 'color':
            pose = self.translate_pose(
                pose, x=-self.tcp_to_ci_cam_pose[0, 3], y=-self.tcp_to_ci_cam_pose[1, 3], z=-distance, frame='self')
        elif cam == 'cloud':
            pose = self.translate_pose(
                pose, x=-self.tcp_to_pc_cam_pose[0, 3], y=-self.tcp_to_pc_cam_pose[1, 3], z=-distance, frame='self')
        else:
            raise Exception('Camera '+str(cam)+' not supported')
        self.movel(pose)

    def pick_part(self, grasp, approach_dist, force_limit=None):
        # Move to approach pose
        self.move_tcp_over_pose(grasp[0], approach_dist, rotate=True)

        # Set gripper width
        self.set_gripper_width(grasp[1])

        # Check force limit
        # TODO

        # Move to pose
        self.movel(grasp[0])

        # Close Gripper
        self.close_gripper()

        # Move back to approach pose
        self.move_tcp_over_pose(grasp[0], approach_dist, rotate=True)

        # Return if object was picked
        return self.close_gripper()

    def back_up_for_view(self, aPose, view_distance):
        """ Get a backed up pose from `aPose` """
        return self.pose_mtrx_to_vec(
            self.translate_pose(aPose,
                                x=-self.tcp_to_ci_cam_pose[0, 3],
                                y=-self.tcp_to_ci_cam_pose[1, 3],
                                z=-view_distance,
                                frame='self')
        )
