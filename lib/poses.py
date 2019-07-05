import numpy as np
import math

class Poses:
    def pose_vec_to_mtrx(self,vec):
        """
        Converts [translation + rotation] vector to a homogeneous transformation matrix.

        Parameters
        ----------
        vector: [6,] list
            Vector.\n
            [x, y, z, rX, rY, rZ]

        Returns
        -------
        matrix : [4,4] ndarray
            Homogeneous Transformation Matrix.\n
            [r11,r12,r13,tx]\n
            [r21,r22,r23,ty]\n
            [r31,r32,r33,tz]\n
            [ 0 , 0 , 0 , 1]
        """
        x, y, z, rx, ry, rz = vec
        theta = np.sqrt(np.square(rx) + np.square(ry) + np.square(rz))
        if theta == 0.0:
            theta =1e-8

        kx = rx/theta
        ky = ry/theta
        kz = rz/theta
        cth = np.cos(theta)
        sth = np.sin(theta)
        vth = 1.0 - np.cos(theta)

        r11 = kx*kx*vth + cth
        r12 = kx*ky*vth - kz*sth
        r13 = kx*kz*vth + ky*sth
        r21 = kx*ky*vth + kz*sth
        r22 = ky*ky*vth + cth
        r23 = ky*kz*vth - kx*sth
        r31 = kx*kz*vth - ky*sth
        r32 = ky*kz*vth + kx*sth
        r33 = kz*kz*vth + cth

        matrix = np.eye(4)
        matrix[:3,-1] = np.array([x,y,z])
        matrix[:3,:3] = np.array([[r11, r12, r13],
                                    [r21, r22, r23],
                                    [r31, r32, r33]])
        return matrix
        
    def pose_mtrx_to_vec(self, matrix):
        """
        Converts homogeneous transformation matrix to a [translation + rotation] vector.

        Parameters
        ----------
        matrix : [4,4] ndarray
            Homogeneous Transformation Matrix.\n
            [r11,r12,r13,tx]\n
            [r21,r22,r23,ty]\n
            [r31,r32,r33,tz]\n
            [ 0 , 0 , 0 , 1]

        Returns
        -------
        vector: [6,] list
            Vector.\n
            [x, y, z, rX, rY, rZ]
        """

        r11 = matrix[0,0]
        r12 = matrix[0,1]
        r13 = matrix[0,2]
        r21 = matrix[1,0]
        r22 = matrix[1,1]
        r23 = matrix[1,2]
        r31 = matrix[2,0]
        r32 = matrix[2,1]
        r33 = matrix[2,2]

        val = (r11+r22+r33-1)/2.0
        while val < -1.0:
            val += 2.0
        while val > 1.0:
            val -= 2.0
        theta = np.arccos(val)

        if theta == 0.0:
            theta =1e-8
        sth = np.sin(theta)
        kx = (r32-r23)/(2*sth)
        ky = (r13-r31)/(2*sth)
        kz = (r21-r12)/(2*sth)

        rv1 = theta*kx
        rv2 = theta*ky
        rv3 = theta*kz

        x = matrix[0,-1] 
        y = matrix[1,-1] 
        z = matrix[2,-1] 

        return [float(x),float(y),float(z),float(rv1),float(rv2),float(rv3)]




    def transform_points(self,points,pose):
        """
        Transform points by a transformation matrix.
        
        Parameters
        ----------
        points: [n,3] ndarray
            Points.
        transform: [4,4] ndarray
            Transformation matrix to be used.\n
            [r11,r12,r13,tx]\n
            [r21,r22,r23,ty]\n
            [r31,r32,r33,tz]\n
            [ 0 , 0 , 0 , 1]
            
        Returns
        -------
        transformed_points: [n,3] ndarray
            The transformed points. 
        """
        points = np.hstack((points,np.ones((points.shape[0],1))))
        points = points.dot(pose.T)[:,:3]
        return points

    def translate_pose(self, pose, x=0.0, y=0.0, z=0.0, frame='base'):
        """
        Alters a transformation matrix, translating it by some amount along \
        each of its axes. 

        Parameters
        ----------
        transform: [4,4] ndarray
            The transformation_matrix.\n
            [r11,r12,r13,tx]\n
            [r21,r22,r23,ty]\n
            [r31,r32,r33,tz]\n
            [ 0 , 0 , 0 , 1]
        x_offset: float (m)
            The amount to translate the transform along its x axis.
        y_offset: float (m)
            The amount to translate the transform along its y axis.
        z_offset: float (m)
            The amount to translate the transform along its z axis.

        Returns
        -------
        transform_translate: [4,4] ndarray
            The original transformation matrix translated by the desired amount along each axis.
        """
        
        if frame == 'base':
            pose_translate = np.copy(pose)
            pose_translate[0,3] = pose_translate[0,3] + x
            pose_translate[1,3] = pose_translate[1,3] + y
            pose_translate[2,3] = pose_translate[2,3] + z
        
        elif frame == 'self':
            end_point = np.hstack((np.eye(3),np.ones((3,1))))
            end_pose = pose.dot(end_point.T)

            x_axis = end_pose[:3,0] - pose[:3,3]
            y_axis = end_pose[:3,1] - pose[:3,3]
            z_axis = end_pose[:3,2] - pose[:3,3]

            translation = x*x_axis + y*y_axis + z*z_axis

            pose_translate = np.copy(pose)
            pose_translate[:3,3] = pose_translate[:3,3]+translation
            
        else:
            raise NameError('Invalid Reference Frame [self, base]')

        return pose_translate

    def rotate_pose(self, pose, rx=0.0, ry=0.0, rz=0.0, frame='base'):
        """
        Alters a transformation matrix, rotating it by some amount around \
        each of its axes. 

        Parameters
        ----------
        transform: [4,4] ndarray
            Transformation matrix.\n
            [r11,r12,r13,tx]\n
            [r21,r22,r23,ty]\n
            [r31,r32,r33,tz]\n
            [ 0 , 0 , 0 , 1]
        rx: float (rad)
            The amount to rotate the transform around its x axis.
        ry: float (rad)
            The amount to rotate the transform around its y axis.
        rz: float (rad)
            The amount to rotate the transform around its z axis.

        Returns
        -------
        transform_rotate: [4,4] ndarray
            The original transformation matrix rotated by the desired amount around each axis.\n
            [r11,r12,r13,tx]\n
            [r21,r22,r23,ty]\n
            [r31,r32,r33,tz]\n
            [ 0 , 0 , 0 , 1]
        """
        rx_mat = np.array(([1,0,0],[0,np.cos(rx),-np.sin(rx)],[0,np.sin(rx),np.cos(rx)]))
        ry_mat = np.array(([np.cos(ry),0,np.sin(ry)],[0,1,0],[-np.sin(ry),0,np.cos(ry)]))
        rz_mat = np.array(([np.cos(rz),-np.sin(rz),0],[np.sin(rz),np.cos(rz),0],[0,0,1]))
        pose_rotate = np.copy(pose)
        if frame == 'self':
            pose_rotate[:3,:3] = pose_rotate[:3,:3].dot(rx_mat).dot(ry_mat).dot(rz_mat)
        elif frame == 'base':
            pose_rotate[:3,:3] = rx_mat.dot(ry_mat).dot(rz_mat).dot(pose_rotate[:3,:3])

        return pose_rotate

    def invert_pose(self, pose):
        """ 
        Return the transform T^{-1} that is the reverse of T. If points are transformed \
        by T, transforming the points then by T^{-1} would return them to their original positions. 
        
        Parameters
        ----------
        transform: [4,4] ndarray
            Transformation matrix.\n
            [r11,r12,r13,tx]\n
            [r21,r22,r23,ty]\n
            [r31,r32,r33,tz]\n
            [ 0 , 0 , 0 , 1]
            
        Returns
        -------
        inv_transform: [4,4] ndarray
            The inverse transformation matrix.\n
            [r11,r12,r13,tx]\n
            [r21,r22,r23,ty]\n
            [r31,r32,r33,tz]\n
            [ 0 , 0 , 0 , 1]
        """

        return np.linalg.inv(pose)

    def is_rotation_mtrx(self, R):
        # Checks if a matrix is a valid rotation matrix.
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6
    
    def rotation_mtrx_to_rpy(self, R) :
        # Calculates rotation matrix to euler angles
        assert(self.is_rotation_mtrx(R))

        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

        singular = sy < 1e-6

        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0

        return np.array([x, y, z])

    
    def rpy_to_rotation_mtrx(self, theta):
        # Calculates Rotation Matrix given euler angles
        R_x = np.array([[1,         0,                  0                   ],
                        [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                        [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                        ])
        R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                        [0,                     1,      0                   ],
                        [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                        ])
        R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                        [math.sin(theta[2]),    math.cos(theta[2]),     0],
                        [0,                     0,                      1]
                        ])
        R = np.dot(R_z, np.dot( R_y, R_x ))
        return R
    
    def cartesian_mtrx_of_poses(self, start_pose, x_range, y_range, z_range, frame='tool', output=None):
        matrix = np.zeros((len(x_range),len(y_range),len(z_range),4,4))
        
        for i, x in enumerate(x_range):
            for j, y in enumerate(y_range):
                for k, z in enumerate(z_range):
                    
                    if frame == 'base':
                        matrix[i,j,k,:] = self.translate_pose(start_pose, x=x, y=y, z=z, frame='base')
                    if frame == 'tool':
                        matrix[i,j,k,:] = self.translate_pose(start_pose, x=x, y=y, z=z, frame='self')

        if output == 'all':
            shape = matrix.shape
            view = self.PC_Viewer()
            for i in range(shape[0]):
                for j in range(shape[1]):
                    for k in range(shape[2]):
                        view.add_axis(matrix[i,j,k,:])
            view.show()

        return matrix

    def cylindircal_mtrx_of_poses(self, start_pose, r_range, theta_range, z_range, frame='tool', cylinder_axis='y', output=None):
        matrix = np.zeros((len(r_range),len(theta_range),len(z_range),4,4))
        
        for i, r in enumerate(r_range):
            for j, theta in enumerate(theta_range):
                for k, z in enumerate(z_range):

                    if frame == 'tool':
                        pose = self.translate_pose(start_pose, z=r_range[0], frame='self')
                        if cylinder_axis == 'y':
                            pose = self.rotate_pose(pose, ry=math.radians(theta), frame='self')
                            matrix[i,j,k,:] = self.translate_pose(pose, y=z, z=-r, frame='self')
                        elif cylinder_axis == 'x':
                            pose = self.rotate_pose(pose, rx=math.radians(theta), frame='self')
                            matrix[i,j,k,:] = self.translate_pose(pose, x=z, z=-r, frame='self')

        if output == 'all':
            shape = matrix.shape
            view = self.PC_Viewer()
            for i in range(shape[0]):
                for j in range(shape[1]):
                    for k in range(shape[2]):
                        view.add_axis(matrix[i,j,k,:])

            view.show()
        return matrix

    def spherical_mtrx_of_poses(self, start_pose, r_range, theta_range, phi_range, frame='tool', output=None):
            matrix = np.zeros((len(phi_range),len(theta_range),len(r_range),4,4))

            for i, phi in enumerate(phi_range):
                for j, theta in enumerate(theta_range):
                    for k, r in enumerate(r_range):

                        if frame == 'tool':
                            pose = self.translate_pose(start_pose, z=r_range[0], frame='self')
                            pose = self.rotate_pose(pose, ry=math.radians(phi), rx=math.radians(theta), frame='self')
                            matrix[i,j,k,:] = self.translate_pose(pose, z=-r, frame='self')

            if output == 'all':
                shape = matrix.shape
                view = self.PC_Viewer()
                for i in range(shape[0]):
                    for j in range(shape[1]):
                        for k in range(shape[2]):
                            view.add_axis(matrix[i,j,k,:])

                view.show()
            return matrix
    
    def align_pose_to_tcp(self,pose,frame='base'):

        if frame == 'base':
            Tbc = self.get_base_to_camera_pose()    
            Tco = self.invert_pose(Tbc).dot(pose)
            rpy = self.rotation_mtrx_to_rpy(Tco[0:3, 0:3])
        elif frame == 'tool':
            rpy = self.rotation_mtrx_to_rpy(pose.copy()[0:3, 0:3])
        else:
            return None

        if abs(math.degrees(rpy[2])) > 90:
            pose = self.rotate_pose(pose,rz=math.radians(180),frame='self')
            pose_new = self.rotate_pose(pose,rx=rpy[0],ry=rpy[1],frame='self')
        else:
            pose_new = self.rotate_pose(pose,rx=-rpy[0],ry=-rpy[1],frame='self')
        
        return pose_new

