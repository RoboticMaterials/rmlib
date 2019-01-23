from scipy.spatial import ConvexHull
from sklearn import linear_model
from sklearn.decomposition import PCA
import numpy as np

class Grasps:
    
    def get_object_transform(self,cloud,vertical=False):
        """
        Finds the origin of the object cloud.

        Parameters
        ----------
        object_cloud: (n,3) ndarray
            Sequence of points in object cloud.

        Return
        ------
        trans: (4,4) ndarray
            The transformation matrix representing the position and orientation of the object. \
            trans[:3,:3] corrisponds to the rotation matrix of the object axis and trans[:3,4] \
            corrisponds to the translation of the origin.
        """

        pca = PCA(n_components=3)
        pca_cloud = pca.fit_transform(cloud)
        hull = ConvexHull(pca_cloud[:,:2])

        max_z = pca_cloud[:,2].min()
        min_z = pca_cloud[:,2].max()
        if pca.components_[2,2] < 0:
            direction = -1
            max_z = pca_cloud[:,2].max()
            min_z = pca_cloud[:,2].min()

        volume_bb = 10000000
        for simplex in hull.simplices:
            u = pca_cloud[simplex[0],:2]
            v = pca_cloud[simplex[1],:2]
            slope = (v[1] - u[1])/(v[0] - u[0])
            theta = np.arctan(slope)
            rot = np.array([[np.cos(theta), -np.sin(theta),0.0],[np.sin(theta), np.cos(theta),0.0],[0.0,0.0,1.0]])
            pc = pca_cloud.dot(rot)
            min_x = pc[:,0].min()
            max_x = pc[:,0].max()
            min_y = pc[:,1].min()
            max_y = pc[:,1].max()
            vol = np.abs(min_x-max_x)*np.abs(min_y-max_y)
            if vol < volume_bb:
                vertices = np.zeros((4,3))
                origin = np.array([min_x,min_y,max_z])
                if (max_x-min_x > max_y-min_y):
                    vertices[0,:] = np.array([max_x,min_y,max_z]) - origin 
                    vertices[1,:] = np.array([min_x,max_y,max_z]) - origin 
                else:
                    vertices[1,:] = np.array([max_x,min_y,max_z]) - origin 
                    vertices[0,:] = np.array([min_x,max_y,max_z]) - origin
                x_offset = (pc[:,0].max() + pc[:,0].min())/2.0
                y_offset = (pc[:,1].max() + pc[:,1].min())/2.0
                vertices[2,:] = np.array([min_x,min_y,min_z]) - origin  
                vertices = vertices.dot(np.linalg.inv(rot))
                translation = np.array([x_offset,y_offset,max_z])
                translation = translation.dot(np.linalg.inv(rot))
                volume_bb = vol

        rot = np.array([vertices[0]/np.linalg.norm(vertices[0]),vertices[1]/np.linalg.norm(vertices[1]),vertices[2]/np.linalg.norm(vertices[2])]).T
        rot_pca = pca.components_.T
        trans = np.eye(4)
        trans[:3,:3]= rot_pca.dot(rot)
        trans[:3,3] = pca.inverse_transform(translation)
        trans[:3,1] = np.cross(trans[:3,2],trans[:3,0])

        if vertical:
            pose = self.convert_transform_to_pose(trans)
            pose[3:5] = 0.0,0.0
            trans = self.convert_pose_to_transform(pose)

        trans = self.rotate_transform(trans,0,0,1.57)
        return trans
    
    def get_object_width(self,object_cloud,transform=None):
        """
        Finds width of object

        Parameters
        ----------
        object_cloud: (n,3) ndarray
            Sequence of points in object cloud.

        Return
        ------
        width: float
            Width of object (m).
        """
        pca = PCA(n_components=3)
        if transform is None:
            object_cloud_pca = pca.fit_transform(object_cloud)
            width = float(object_cloud_pca[:,1].max() - object_cloud_pca[:,1].min())
        else:
            object_cloud_AA = self.transform_points(object_cloud,np.linalg.inv(transform))
            width = object_cloud_AA[:,0].max() - object_cloud_AA[:,0].min()

        return width
    
    def shift_transform_to_grasp(self,cloud,object_transform):
        """
        Shifts transform down from the top of an object closer to the surface (table)
        """
        
        x_y_coords_pts = cloud[:,:2]
        z_coord_pts = cloud[:,-1]

        ransac = linear_model.RANSACRegressor()
        ransac.fit(x_y_coords_pts,z_coord_pts)
        
        transforms_z_on_surface = (object_transform[:2,3].T).dot(ransac.estimator_.coef_) + ransac.estimator_.intercept_
                
        shift = transforms_z_on_surface - object_transform[2,3]
        
        shifted_transform = self.translate_transform(object_transform,0,0,shift*2/3)
        
        return shifted_transform

    def get_gripper_boxes(self,transform,width):
        """
        Gives the vertices of the bounding boxes of the gripper for the proposed grasp pose. 

        Parameters
        ----------
        transform: (4,4) ndarray
            transform[:3,:3] corrisponds to the rotation matrix of the grasp orientation and transform[:3,4] \
            corrisponds to the translation of the grasp. This transform can be generated from get_object_transform().
        width: float
            Width of the object cloud for the grasp.

        Returns
        -------
        gripper: ndarray of 4 (8,3) ndarrays
            The 4 arrays corrispond to the 4 boxes that describe the shape of the gripper. 
            z
        """

        transform = self.rotate_transform(transform,0,0,1.57)
        start_point = np.hstack((np.zeros((3,3)), np.ones((3,1))))
        end_point = np.hstack((np.eye(3), np.ones((3,1))))

        start_trans = transform.dot(start_point.T)
        end_trans = transform.dot(end_point.T)

        x_vec = end_trans[:3,0] - start_trans[:3,0]
        x_vec = x_vec/np.linalg.norm(x_vec)

        y_vec = end_trans[:3,1] - start_trans[:3,1]
        y_vec = y_vec/np.linalg.norm(y_vec)

        z_vec = end_trans[:3,2] - start_trans[:3,2]
        z_vec = z_vec/np.linalg.norm(z_vec)

        pos = transform[:3,3]

        fingerbox0 = np.zeros((8,3))
        fingerbox1 = np.zeros((8,3))
        box0 = np.zeros((8,3))
        box1 = np.zeros((8,3))
        box2 = np.zeros((8,3))
        box3 = np.zeros((8,3))

        fingerbox0[0] = pos + (0.007*x_vec) - (((width/2.0)+0.003)*y_vec)
        fingerbox0[1] = pos + (0.007*x_vec) - ((width/2.0)*y_vec)
        fingerbox0[2] = fingerbox0[0] - ((0.04+self.finger_offset)*z_vec)
        fingerbox0[3] = fingerbox0[1] - ((0.04+self.finger_offset)*z_vec)
        fingerbox0[4] = fingerbox0[0] - (0.016*x_vec)
        fingerbox0[5] = fingerbox0[1] - (0.016*x_vec)
        fingerbox0[6] = fingerbox0[2] - (0.016*x_vec)
        fingerbox0[7] = fingerbox0[3] - (0.016*x_vec)

        fingerbox1[0] = pos + (0.007*x_vec) + (((width/2.0)+0.003)*y_vec)
        fingerbox1[1] = pos + (0.007*x_vec) + ((width/2.0)*y_vec)
        fingerbox1[2] = fingerbox1[0] - ((0.04+self.finger_offset)*z_vec)
        fingerbox1[3] = fingerbox1[1] - ((0.04+self.finger_offset)*z_vec)
        fingerbox1[4] = fingerbox1[0] - (0.016*x_vec)
        fingerbox1[5] = fingerbox1[1] - (0.016*x_vec)
        fingerbox1[6] = fingerbox1[2] - (0.016*x_vec)
        fingerbox1[7] = fingerbox1[3] - (0.016*x_vec)

        box0[0] = pos + (0.007*x_vec) - (((width/2.0)+0.015)*y_vec)
        box0[1] = pos + (0.007*x_vec) - ((width/2.0)*y_vec)
        box0[2] = box0[0] - ((0.04+self.finger_offset)*z_vec)
        box0[3] = box0[1] - ((0.04+self.finger_offset)*z_vec)
        box0[4] = box0[0] - (0.016*x_vec)
        box0[5] = box0[1] - (0.016*x_vec)
        box0[6] = box0[2] - (0.016*x_vec)
        box0[7] = box0[3] - (0.016*x_vec)

        box1[0] = pos + (0.007*x_vec) + (((width/2.0)+0.015)*y_vec)
        box1[1] = pos + (0.007*x_vec) + ((width/2.0)*y_vec)
        box1[2] = box1[0] - ((0.04+self.finger_offset)*z_vec)
        box1[3] = box1[1] - ((0.04+self.finger_offset)*z_vec)
        box1[4] = box1[0] - (0.016*x_vec)
        box1[5] = box1[1] - (0.016*x_vec)
        box1[6] = box1[2] - (0.016*x_vec)
        box1[7] = box1[3] - (0.016*x_vec)

        pivot_height_1 = ((width/4.0)+0.028)
        pivot_height_2 = (((0.055**2.0)-((width/2.0)+0.008)**2.0)**0.5)-0.017+self.finger_offset
        pivot_width_from_center = ((width/2.0)+0.045)

        box2[0] = pos + (0.01*x_vec) - (pivot_width_from_center*y_vec) - ((0.04+self.finger_offset)*z_vec)
        box2[1] = box2[0] + (2.0*pivot_width_from_center*y_vec)
        box2[2] = box2[0] - ((pivot_height_1+pivot_height_2)*z_vec)
        box2[3] = box2[2] + (2.0*pivot_width_from_center*y_vec)
        box2[4] = box2[0] - (0.02*x_vec)
        box2[5] = box2[1] - (0.02*x_vec)
        box2[6] = box2[2] - (0.02*x_vec)
        box2[7] = box2[3] - (0.02*x_vec)

        box3[0] = pos + (0.050*x_vec) - (0.05*y_vec) - ((0.047+pivot_height_2)*z_vec)
        box3[1] = pos + (0.050*x_vec) + (0.05*y_vec) - ((0.047+pivot_height_2)*z_vec)
        box3[2] = box3[0] - (0.14*z_vec)
        box3[3] = box3[1] - (0.14*z_vec)
        box3[4] = box3[0] - (0.09*x_vec)
        box3[5] = box3[1] - (0.09*x_vec)
        box3[6] = box3[2] - (0.09*x_vec)
        box3[7] = box3[3] - (0.09*x_vec)

        return np.array((box0,box1,box2,box3)) , np.array((fingerbox0,fingerbox1))