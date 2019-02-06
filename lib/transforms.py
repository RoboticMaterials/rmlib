import numpy as np
import math

class Transforms:

    def pose_trans(self,pose_1,pose_2):
        """
        Combines pose vectors to get the first pose relative to a secondary reference frame. For example, \
        if you have the pose of the object relative to the camera and the pose of the camera relative to the \
        robot, pose_trans(camera_pose,robot_pose) will give you the pose of the object relative to the robot. 

        Parameters
        ----------
        pose_1: (1,6) ndarray
            First pose vector to be added relative to second pose. 
        pose_2: (1,6) ndarray
            Second pose vector
        """

        transform_1 = self.convert_pose_to_transform(pose_1)
        transform_2 = self.convert_pose_to_transform(pose_2) 
        transform_trans = transform_1.dot(transform_2)
        return self.convert_transform_to_pose(transform_trans)

    def convert_pose_to_transform(self,pose):
        """
        Converts a pose vector to a transformation matrix.

        Parameters
        ----------
        pose: (1,6) ndarray
            Pose vector.

        Returns
        -------
        transform :(4,4) ndarray
            Transformation matrix.
        """
        x, y, z, rx, ry, rz = pose
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

        transform = np.eye(4)
        transform[:3,-1] = np.array([x,y,z])
        transform[:3,:3] = np.array([[r11, r12, r13],
                                    [r21, r22, r23],
                                    [r31, r32, r33]])

        return transform

    def convert_transform_to_pose(self,transform,x_offset=0.0,y_offset=0.0,z_offset=0.0,is_object=False,bound=False):
        """
        Converts a pose vector to a transformation matrix.

        Parameters
        ----------
        transform :(4,4) ndarray
            Transformation matrix.
        x_offset/y_offset/z_offset: float
            The offsets from the position if neccessary.
        is_object: bool
            Rotates 90 degrees from principal axis to pick up object.
        bound: bool
            If True, bound will restrict the gripper from rotating too far. If you care about the orientation \
            of the object, set bound to False. 

        Returns
        -------
        pose: (1,6) ndarray
            Pose vector.
        """
        r11 = transform[0,0]
        r12 = transform[0,1]
        r13 = transform[0,2]
        r21 = transform[1,0]
        r22 = transform[1,1]
        r23 = transform[1,2]
        r31 = transform[2,0]
        r32 = transform[2,1]
        r33 = transform[2,2]

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

        if is_object:
            rv3 = rv3 + 1.5708
            if bound:
                if rv3 > 1.5708:
                    rv3 -= 3.14159
                elif rv3 < -1.5708:
                    rv3 += 3.14159

        x = transform[0,-1] 
        y = transform[1,-1] 
        z = transform[2,-1] 

        return [float(x+x_offset),float(y+y_offset),float(z+z_offset),float(rv1),float(rv2),float(rv3)]

    def transform_points(self,points,transform):
        points = np.hstack((points,np.ones((points.shape[0],1))))
        points = points.dot(transform.T)[:,:3]
        return points

    def transform_points_inv(self,points,transform):
        points -= transform[:3,-1]
        rot = transform[:3,:3]
        points = points.dot(rot)[:,:3]
        return points

    def transform_transform(self,transform_original,transform):
        return transform.dot(transform_original)

    def transform_pose(self,pose,transform):
        pose_transform = convert_pose_to_transform(pose)
        new_pose_transform = transform_transform(pose_transform,transform)
        return convert_transform_to_pose(new_pose_transform)

    def translate_transform(self,transform,x_offset=0.0,y_offset=0.0,z_offset=0.0):
        """
        This function alters a transformation matrix, translating it by some amount along \
        each of its axes. 

        Parameters
        ----------
        transform: (4,4) ndarray
        x_offset: float (m)
            The amount to translate the transform along its x axis.
        y_offset: float (m)
            The amount to translate the transform along its y axis.
        z_offset: float (m)
            The amount to translate the transform along its z axis.

        Returns
        -------
        transform_translate: (4,4) ndarray
            The original transformation matrix translated by the desired amount along each axis.
        """
        end_point = np.hstack((np.eye(3),np.ones((3,1))))
        end_trans = transform.dot(end_point.T)

        x_axis = end_trans[:3,0] - transform[:3,3]
        y_axis = end_trans[:3,1] - transform[:3,3]
        z_axis = end_trans[:3,2] - transform[:3,3]

        translation = x_offset*x_axis + y_offset*y_axis + z_offset*z_axis

        transform_translate = np.copy(transform)
        transform_translate[:3,3] = transform_translate[:3,3]+translation
        return transform_translate

    def translate_pose(self,pose,x_offset=0.0,y_offset=0.0,z_offset=0.0):
        transform = self.convert_pose_to_transform(pose)
        end_point = np.hstack((np.eye(3),np.ones((3,1))))
        end_trans = transform.dot(end_point.T)

        x_axis = end_trans[:3,0] - transform[:3,3]
        y_axis = end_trans[:3,1] - transform[:3,3]
        z_axis = end_trans[:3,2] - transform[:3,3]

        translation = x_offset*x_axis + y_offset*y_axis + z_offset*z_axis

        transform_translate = np.copy(transform)
        transform_translate[:3,3] = transform_translate[:3,3]+translation
        return self.convert_transform_to_pose(transform_translate)

    def rotate_transform(self,transform,rx=0.0,ry=0.0,rz=0.0):
        """
        This function alters a transformation matrix, rotating it by some amount around \
        each of its axes. 

        Parameters
        ----------
        transform: (4,4) ndarray
        rx: float (rad)
            The amount to rotate the transform around its x axis.
        ry: float (rad)
            The amount to rotate the transform around its y axis.
        rz: float (rad)
            The amount to rotate the transform around its z axis.

        Returns
        -------
        transform_rotate: (4,4) ndarray
            The original transformation matrix rotated by the desired amount around each axis.
        """
        rx_mat = np.array(([1,0,0],[0,np.cos(rx),-np.sin(rx)],[0,np.sin(rx),np.cos(rx)]))
        ry_mat = np.array(([np.cos(ry),0,np.sin(ry)],[0,1,0],[-np.sin(ry),0,np.cos(ry)]))
        rz_mat = np.array(([np.cos(rz),-np.sin(rz),0],[np.sin(rz),np.cos(rz),0],[0,0,1]))

        transform_rotate = np.copy(transform)
        transform_rotate[:3,:3] = transform_rotate[:3,:3].dot(rx_mat).dot(ry_mat).dot(rz_mat)

        return transform_rotate

    def rotate_pose(self,pose,rx=0.0,ry=0.0,rz=0.0):
        transform = self.convert_pose_to_transform(pose)
        rx_mat = np.array(([1,0,0],[0,np.cos(rx),-np.sin(rx)],[0,np.sin(rx),np.cos(rx)]))
        ry_mat = np.array(([np.cos(ry),0,np.sin(ry)],[0,1,0],[-np.sin(ry),0,np.cos(ry)]))
        rz_mat = np.array(([np.cos(rz),-np.sin(rz),0],[np.sin(rz),np.cos(rz),0],[0,0,1]))

        transform_rotate = np.copy(transform)
        transform_rotate[:3,:3] = transform_rotate[:3,:3].dot(rx_mat).dot(ry_mat).dot(rz_mat)

        return self.convert_transform_to_pose(transform_rotate)

    def unit_vec(self,vec):
        """ Return the unit vector in the direction of 'vec' """
        vLen = np.linalg.norm(vec)
        if vLen == 0: # Avoid DIV0 errors
            return vec # Unit vec undefined, return vector unchanged
        else:
            return np.divide(vec,vLen)

    def invert_transform(self,transform):
        """ Return the transform T^{-1} that is the reverse of T """
        R = transform[0:3, 0:3]
        Rinv = np.linalg.inv(R)
        d = transform[0:3, 3]
        dInv = np.dot(Rinv, -d)
        rtnMatx = np.eye(4)
        rtnMatx[0:3, 0:3] = Rinv
        rtnMatx[0:3, 3] = dInv
        return rtnMatx

    def relative_transform(self,AtoB, AtoC):
        """ For homogeneous transforms 'AtoB' and 'AtoC' , Return the transform B to C """
        return np.dot(AtoC, invert_transform(AtoB))

    # Checks if a matrix is a valid rotation matrix.
    def is_rotation_matrix(self, R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    # Calculates rotation matrix to euler angles
    def rotation_matrix_to_RPY(self, R) :
        assert(self.is_rotation_matrix(R))

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

    #Calculates Rotation Matrix given euler angles
    def RPY_to_rotation_matrix(self, theta) :
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
    
    #Convert pixles to space
    def pxls_to_spc(self, pxls_dist, z_cam):
        return ( pxls_dist / 1280) * z_cam * math.tan(math.radians(69.4/2)) * 2

    #Convert space to pixles
    def spc_to_pxls(self, spc_dist, z_cam):
        return spc_dist * 1280 / (z_cam * math.tan(math.radians(69.4/2)) * 2)

    
    
