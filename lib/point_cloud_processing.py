import numpy as np
import open3d 
from sklearn import linear_model
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
from scipy.spatial import ConvexHull

class Point_Cloud_Processing:
    
    def voxel_grid(self,cloud, leaf_size=0.005):
        """
        Compresses cloud using voxel grid.

        Parameters
        ----------
        cloud: (n,3) ndarray
            Sequence of points in cloud.
        leaf_size: float
            Leaf size of voxel grid.

        Return
        ------
        cloud_vg: (k,3) ndarray
            Sequence of points of compressed cloud.
        """
        if leaf_size >= 0.005:
            min_x, min_y, min_z = cloud.min(axis=0)
            max_x, max_y, max_z = cloud.max(axis=0)
            x = np.arange(min_x, max_x, leaf_size)
            y = np.arange(min_y, max_y, leaf_size)
            z = np.arange(min_z, max_z, leaf_size)

            voxel_x = np.clip(np.searchsorted(x,cloud[:,0]) - 1, 0, x.shape[0])
            voxel_y = np.clip(np.searchsorted(y,cloud[:,1]) - 1, 0, y.shape[0])
            voxel_z = np.clip(np.searchsorted(z,cloud[:,2]) - 1, 0, z.shape[0])

            voxel_id = np.ravel_multi_index([voxel_x, voxel_y, voxel_z],[x.shape[0],y.shape[0],z.shape[0]])

            sort_idx = np.argsort(voxel_id)
            voxel_id_sorted = voxel_id[sort_idx]
            first_id = np.concatenate(([True], voxel_id_sorted[1:] != voxel_id_sorted[:-1]))
            ids_count = np.diff(np.nonzero(first_id)[0])
            ids_idxs = np.split(sort_idx, np.cumsum(ids_count))

            cloud_vg = np.zeros((len(ids_idxs),3))
            for i, idxs in enumerate(ids_idxs): 
                cloud_vg[i] = np.mean(cloud[idxs],axis=0)

        else:
            pcd = open3d.PointCloud()
            pcd.points = open3d.Vector3dVector(cloud)
            downpcd = open3d.voxel_down_sample(pcd,voxel_size=leaf_size)
            cloud_vg = np.asarray(downpcd.points)

        return cloud_vg

    def remove_planar_surface(self,cloud,remove_tolerance=0.0025,camera_transform=None):
        """
        Removes surface or 'table' from cloud.

        Parameters
        ----------
        cloud: (n,3) ndarray
            Sequence of points in cloud.
        remove_tolerance: float
            Thickness tolerance for surface removal (m).
        camera_transform: (4,4) ndarray
            The camera transform. If local, this transform will be none and this function will generate it's best \
            guess based on RANSAC.
        Return
        ------
        cloud_nt: (k,3) ndarray
            Sequence of points of original cloud with points corrisponding to surface removed.
        """
        surface_est = self.get_planar_surface_estimate(cloud.copy(),camera_transform)[:,2]
        dist = cloud[:,2]-surface_est
        not_table_idxs = np.where(np.logical_and(np.abs(dist) > remove_tolerance, dist < 0.0))
        cloud_nt = cloud[not_table_idxs]

        return cloud_nt

    def get_planar_surface_estimate(self,cloud,camera_transform=None):
        """
        Gets an array of z values corrisponding to cloud mapped to the z value of the table estimate. 

        Parameters
        ----------
        cloud: (n,3) ndarray
            The compressed point cloud.
        camera_transform: (4,4) ndarray
            The camera transform. If working on local machine, this transform will be none and this \
            function will generate it's best guess based on RANSAC.

        Returns
        -------
        table_estimate: (n,1) ndarray
            1d array of z values corrisponding to the cloud mapped to the z values of the estimated table.
        """

        if camera_transform is None:
            x_y_coords_pts = cloud[:,:2]
            z_coord_pts = cloud[:,-1]

            ransac = linear_model.RANSACRegressor()
            ransac.fit(x_y_coords_pts,z_coord_pts)

            surface_est = x_y_coords_pts.dot(ransac.estimator_.coef_) + ransac.estimator_.intercept_
        else:
            cloud_glob = self.transform_points(cloud,camera_transform)
            surface_glob = np.copy(cloud_glob)
            surface_glob[:,2] = 0.0

            surface_loc = self.transform_points(surface_glob,np.linalg.inv(camera_transform))
            surface_est = surface_loc[:,2]

        plane_cloud = cloud
        plane_cloud[:,2] = surface_est

        return plane_cloud

    def get_distance_estimate(self,cloud,camera_transform=None):
        surface_est = self.get_planar_surface_estimate(cloud,camera_transform)[:,2]
        return np.average(surface_est)

    def sort_clouds_height(self,clouds):
        """
        Sorts clouds based on their maximum height.

        Parameters
        ----------
        clouds: list of k (n,3) ndarrays corrisponding to clouds.

        Returns
        -------
        sorted_clouds: list of k (n,3) ndarrays corrisponding to clouds sorted from highest to lowest maximum height.

        """

        max_z = np.array([cloud[:][2].max() for cloud in clouds])
        return [clouds[idx] for idx in np.argsort(max_z)]

    def sort_clouds_size(self,clouds):
        """
        Sorts clouds based on their size.

        Parameters
        ----------
        clouds: list of k (n,3) ndarrays corrisponding to clouds.

        Returns
        -------
        sorted_clouds: list of k (n,3) ndarrays corrisponding to clouds sorted from largest to smallest.

        """

        clouds.sort(key=lambda x: len(x))
        clouds.reverse()
        return clouds

    def crop_cloud(self,cloud,box):
        cropped_cloud = cloud[np.where(self.is_point_in_box(box,cloud))]
    
        return cropped_cloud

    def get_object_clouds(self,cloud, search_radius=0.005,min_cluster_size=100):
        """
        Segments cloud via DBSCAN. DBSCAN looks for points within radius to add to segmented cloud\
        and subsequently uses that new point to spread.

        Parameters
        ---------
        cloud: (n,3) ndarray
            Sequence of points in cloud.
        search_radius: float
            Radius that DBSCAN uses to search.
        min_cluster_size: int
            Minimum amounts of points in a segment to be considered an object.

        Returns
        -------
        object_clouds: sequence of (m,(k,3)) arrays
            A sequence of cloud arrays corrisponding to each object found.
        """

        db = DBSCAN(eps=search_radius).fit(cloud)

        objects_ids = np.unique(db.labels_)
        num_objects = objects_ids.shape[0]

        if objects_ids[0] == -1:
            cloud = cloud[np.where(db.labels_ != -1)]
            db.labels_ = db.labels_[np.where(db.labels_ != -1)] 

        objects_clouds = []
        for i in objects_ids:
            object_idxs = np.where(db.labels_ == i)
            if len(object_idxs[0]) < min_cluster_size:
                cloud = cloud[np.where(db.labels_ != i)]
                db.labels_ = db.labels_[np.where(db.labels_ != i)]
            else:
                objects_clouds.append(cloud[object_idxs])

        return objects_clouds

    def get_object_from_mask(self,mask,depth,height,vertical=False):
        """
        Converts a mask from an IR image to a point cloud by overlaying the image mask onto the depth image. 

        Parameters
        ----------
        rsc: dev
            Camera device.
        mask: (720,1280) ndarray
            Mask should be zeros except for at the indeces of the desired object. 
        depth: ndarray
            Depth data.
        height: float (m)
            Height of object

        Returns
        -------
        object: (n,3) ndarray
            Cloud for object.
        """

        ransac = linear_model.RANSACRegressor()

        ring_depth_idxs = np.where(np.logical_and((mask!=0),(depth!=0.0)))
        ring_mask_idxs = np.where(mask!=0.0)

        offset = False

        X = np.vstack(ring_depth_idxs).T
        X2 = np.vstack(ring_mask_idxs).T
        y = depth[ring_depth_idxs]
        if len(y)<500:
            depth_idxs = np.where(depth!=0)
            perm = np.random.permutation(int((np.vstack(depth_idxs).shape)[1]/4))
            X = (np.vstack(depth_idxs).T)[perm]
            y = (depth[depth_idxs].T - height*1000)[perm]
            offset = True

        if vertical:
            y2 = int(np.average(y))
        else:
            ransac.fit(X,y)
            y2 = ransac.predict(X2)

        if offset:
            y2 -= height * 10000

        ring_filled_image = np.zeros(mask.shape)
        ring_filled_image[ring_mask_idxs] = y2

        ring_cloud = self.convert_depth_image_to_point_cloud(ring_filled_image)

        return ring_cloud
    
    def get_bounding_box(self,cloud):
        """
        Finds bounding box of object cloud.

        Parameters
        ----------
        cloud: (n,3) ndarray
            Sequence of points in object cloud.

        Return
        ------
        vertices: (8,3) ndarray
            Coordinates for vertices of the object bouding box\
            Order:\
            [min_x,min_y,max_z]
            [max_x,min_y,max_z]
            [min_x,max_y,max_z]
            [max_x,max_y,max_z]
            [min_x,min_y,min_z]
            [max_x,min_y,min_z]
            [min_x,max_y,min_z]
            [max_x,max_y,min_z]
        dimensions: (3) ndarray
            The dimensions of the bounding box (x,y,z)
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
                vertices = np.zeros((8,3))
                vertices[0,:] = [min_x,min_y,max_z]
                vertices[1,:] = [max_x,min_y,max_z]
                vertices[2,:] = [min_x,max_y,max_z]
                vertices[3,:] = [max_x,max_y,max_z]
                vertices[4,:] = [min_x,min_y,min_z]
                vertices[5,:] = [max_x,min_y,min_z]
                vertices[6,:] = [min_x,max_y,min_z]
                vertices[7,:] = [max_x,max_y,min_z]
                vertices = vertices.dot(np.linalg.inv(rot))
                volume_bb = vol

            dim1 = np.linalg.norm(vertices[1]-vertices[0])
            dim2 = np.linalg.norm(vertices[2]-vertices[0])
            dim3 = np.linalg.norm(vertices[4]-vertices[0])
            if dim1>dim2:
                bb_dim = [dim1,dim2,dim3]
            else:
                bb_dim = [dim2,dim1,dim3]

        return pca.inverse_transform(vertices), bb_dim
    
    def extrude_bounding_rect(self,rectangle,height):
        """
        Extrudes bounding rectangle down a specified height relative to the orientation of the rectangle \
        to make a bounding box.

        Parameters
        ----------
        rectangle: (4,3) ndarray
            Corrdinates of vertices for bounding rectangle.
        height: float
            Height of extrusion (m).

        Return
        ------
        bounding_box: (8,3) ndarray
            Coordinates for vertices of the object bouding box\
            Order:\
            [min_x,min_y,max_z]
            [max_x,min_y,max_z]
            [min_x,max_y,max_z]
            [max_x,max_y,max_z]
            [min_x,min_y,min_z]
            [max_x,min_y,min_z]
            [min_x,max_y,min_z]
            [max_x,max_y,min_z]
        """

        edge1 = np.subtract(rectangle[1],rectangle[0])
        edge2 = np.subtract(rectangle[2],rectangle[0])

        edge1_norm = np.divide(edge1,np.linalg.norm(edge1))
        edge2_norm = np.divide(edge2,np.linalg.norm(edge2))
        transform = np.cross(edge1_norm,edge2_norm)
        transform = transform/np.linalg.norm(transform)
        if transform[2]<0:
            transform = -transform
        vertices = np.zeros((8,3))
        vertices[0:4,:] = rectangle
        for i,point in enumerate(rectangle):
            vertices[i+4,:] = np.add(point,transform*height)
        return vertices

    def compare_dimensions(self,object_cloud,model_dims,tolerance,transform=None,check_height=False,smaller_than=False):
        """
        Compares the dimensions of a bounding rectangle to the dimensions os a specified \
        object. Use for designating valid objects.

        Parameters
        ----------
        object_cloud: (n,3) ndarray
            The point cloud of the object that is being compared.
        model_dims: (1,3) ndarray
            The dimensions of the ideal object [length,width,height] where length>width. \
            Note that if any of the dimensions is set to -1 this function will ignore that dimension \
            in its comparison.
        tolerance: float
            Tolerance of object dimension comparison (m).
        smaller_than: bool
            If this parameter is True, this function will return true if the object cloud is \
            smaller than the model dimensions reguardless of the tolerance. (but if it is larger \
            it still must be within the tolerance)
        Return
        ------
        match: bool
            True if object dimensions are within the tolerance and if the aspect ratio is reasonable.
        """

        model_length = model_dims[0]
        model_width = model_dims[1]
        model_height = model_dims[2]

        match = False
        model_asp = model_width/model_length

        if transform is None:
            _,obj_dims = self.get_bounding_box(object_cloud)
        else:
            object_cloud_transformed = self.transform_points(object_cloud,np.linalg.inv(transform))
            obj_dims = np.array([object_cloud_transformed[:,0].max()-object_cloud_transformed[:,0].min(),\
                                   object_cloud_transformed[:,1].max()-object_cloud_transformed[:,1].min(),\
                                   object_cloud_transformed[:,2].max()-object_cloud_transformed[:,2].min()])

        obj_length = np.array(obj_dims[:2]).max()
        obj_width = np.array(obj_dims[:2]).min()
        obj_height = np.array(obj_dims[2])

        obj_asp = obj_width/obj_length

        if model_length==-1:
            obj_length = -1
            model_asp = obj_asp
        if model_width==-1:
            obj_width = -1
            model_asp = obj_asp

        height_check = True
        if check_height:
            height_check = False
            if smaller_than:
                if obj_height-model_height <= tolerance:
                    height_check = True
            else:
                if abs(obj_height-model_height) <= tolerance:
                    height_check = True

        if smaller_than:
            if (obj_width-model_width <= tolerance) and \
               (obj_length-model_length <= tolerance) and \
               (abs(obj_asp-model_asp) <= 0.4) and \
               (height_check):
                   match = True
        else:
            if (abs(obj_width-model_width) <= tolerance) and \
               (abs(obj_length-model_length) <= tolerance) and \
               (abs(obj_asp-model_asp) <= 0.4) and \
               (height_check):
                   match = True
        return match

    def project_point_onto_line(self,x0,y0,z0,x1,y1,z1,x2,y2,z2):
        """
        Projects point onto a vector.

        Parameters
        ----------
        x0: (n,1) array
            X value of point.
        y0: (n,1) array
            Y value of point.
        x1: float
            X value of vector start.
        y1: float
            Y value of vector start.
        x2: float
            X value of vector end.
        y2: float
            Y value of vector end.

        Return
        ------
        vector: (n,2) array
            Array of projected vectors.

        References
        ----------
        reference https://en.wikipedia.org/wiki/Vector_projection

        """

        a = np.vstack([x1,y1,z1]).T
        b = np.vstack([x2,y2,z2]).T
        p = np.vstack([x0,y0,z0]).T

        ap = p-a
        ab = b-a
        return a + np.sum(ap*ab,axis=1)[np.newaxis].T/np.sum(ab*ab,axis=1)[np.newaxis].T * ab

    def check_projection_onto_line_segment(self,x0,y0,z0,x1,y1,z1,x2,y2,z2):
        """
        Finds indexes of valid point projections.

        Parameters
        ----------
        x0: (n,1) array
            X value of point.
        y0: (n,1) array
            Y value of point.
        x1: float
            X value of vector start.
        y1: float
            Y value of vector start.
        x2: float
            X value of vector end.
        y2: float
            Y value of vector end.

        Return
        ------
        idxs: (1,k) ndarray
            Array of indices corrisponding to valid projections.
        """
        proj = self.project_point_onto_line(x0,y0,0,x1,y1,0,x2,y2,0)
        if x1 < x2:
            valid_idx = np.where(np.logical_and(proj[:,0] > x1, proj[:,0] < x2))
        elif x1 > x2:
            valid_idx = np.where(np.logical_and(proj[:,0] < x1, proj[:,0] > x2))
        return valid_idx

    def point_line_dist(self,x0,y0,x1,y1,x2,y2,check=False):
        """
        Finds minimum distance from point to vector.

        Parameters
        ----------
        x0: (n,1) array
            X value of point.
        y0: (n,1) array
            Y value of point.
        x1: float
            X value of vector start.
        y1: float
            Y value of vector start.
        x2: float
            X value of vector end.
        y2: float
            Y value of vector end.

        Return
        ------
        distance: float
            Minimum distance from point to vector (m).

        References
        ----------
        reference http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
        """
        if check == False:
            dist = np.abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1))/np.sqrt((x2-x1)**2+(y2-y1)**2)
        elif check == True:
            valid_idxs = self.check_projection_onto_line_segment(x0,y0,x1,y1,x2,y2)
            dist = np.abs((x2-x1)*(y1-y0[valid_idxs]) - (x1-x0[valid_idxs])*(y2-y1))/np.sqrt((x2-x1)**2+(y2-y1)**2)
        return dist


    def get_transform_for_CABB(self,box):
        """
        Find the transform matrix to make a list of points center alligned (center of the box at the origin)

        Parameters
        ---------
        box: (8,3) ndarray
            Vertices of the bounding box for allignment.

        Returns
        -------
        trans_mat: (4,4) ndarray
            [r11,r12,r13,tx]
            [r21,r22,r23,ty]
            [r31,r32,r33,tz]
            [ 0 , 0 , 0 , 1]
        """

        rot = np.array(((box[1]-box[0])/np.linalg.norm(box[1]-box[0]),\
                        (box[2]-box[0])/np.linalg.norm(box[2]-box[0]),\
                        (box[4]-box[0])/np.linalg.norm(box[4]-box[0])))
        box_rot = box.dot(rot.T)
        transl = (box_rot[0] +\
                      0.5*(box_rot[1]-box_rot[0]) +\
                      0.5*(box_rot[2]-box_rot[0]) +\
                      0.5*(box_rot[4]-box_rot[0]))
        trans_mat = np.eye(4)
        trans_mat[:3,:3] = rot
        trans_mat[:3,3] = transl
        return trans_mat

    def get_transform_for_AABB(self,box):
        """
        Find the transform matrix to make a list of points axis alligned (corner of the box at the origin)

        Parameters
        ---------
        box: (8,3) ndarray
            Vertices of the bounding box for allignment.

        Returns
        -------
        trans_mat: (4,4) ndarray
            [r11,r12,r13,tx]
            [r21,r22,r23,ty]
            [r31,r32,r33,tz]
            [ 0 , 0 , 0 , 1]
        """

        rot = np.array(((box[1]-box[0])/np.linalg.norm(box[1]-box[0]),\
                        (box[2]-box[0])/np.linalg.norm(box[2]-box[0]),\
                        (box[4]-box[0])/np.linalg.norm(box[4]-box[0])))

        trans_mat = np.eye(4)
        trans_mat[:3,:3] = rot
        trans_mat[:3,3] = box[0,:]
        return trans_mat

    def get_rotation_matrix_for_line(self,lA,lB):
        """
        Finds the rotation matrix to allign lA to lB.

        Parameters
        ----------
        lA: (1,3) ndarray
            The vector (starting at the origin) that you want to rotate.
        lB: (1,3) ndarray
            The vector (starting at the origin) that you want to allign to.

        Returns
        -------
        trans_mat: (4,4) ndarray
            [r11,r12,r13,tx]
            [r21,r22,r23,ty]
            [r31,r32,r33,tz]
            [ 0 , 0 , 0 , 1]
        """

        lA_norm = lA/np.linalg.norm(lA)
        lB_norm = lB/np.linalg.norm(lB)

        lC = np.cross(lA_norm,lB_norm) # Orthagonal vector to A & B (axis of rotation)
        Cx,Cy,Cz = lC/np.linalg.norm(lC)
        theta = np.arccos(lA_norm.dot(lB_norm))

        sint = np.sin(theta)
        cost = np.cos(theta)

        rot_mat = np.array(([(cost)+(Cx*Cx*(1-cost)),   (Cx*Cy*(1-cost))-(Cz*sint),(Cx*Cz*(1-cost))+(Cy*sint)],\
                            [(Cy*Cx*(1-cost))+(Cz*sint),(cost)+(Cy*Cy*(1-cost)),   (Cy*Cz*(1-cost))-(Cx*sint)],\
                            [(Cz*Cx*(1-cost))-(Cy*sint),(Cz*Cy*(1-cost))+(Cx*sint),(cost)+(Cz*Cz*(1-cost))]))

        return np.linalg.inv(rot_mat)

    def do_boxes_collide(self,A,B):
        """
        Checks collision between two rectangular prisms. Find the plane of separation with the \
        assumption that it is either parallel to one of the faces of one of the boxes or that it \
        is orthagonal to the cross product of two of the axis of the boxes.

        Parameters
        ----------
        A: (8,3) ndarray
            The vertices for the first box.
        B: (8,3) ndarray
            The vertices for the second box.

        Returns
        -------
        collision: bool
            Returns True if the boxes collide.
        """

        x_axis = np.array((1,0,0))

        Ax = np.array(A[1]-A[0])/np.linalg.norm(A[1]-A[0])
        Ay = np.array(A[4]-A[0])/np.linalg.norm(A[4]-A[0])
        Az = np.array(A[2]-A[0])/np.linalg.norm(A[2]-A[0])

        Bx = np.array(B[1]-B[0])/np.linalg.norm(B[1]-B[0])
        By = np.array(B[4]-B[0])/np.linalg.norm(B[4]-B[0])
        Bz = np.array(B[2]-B[0])/np.linalg.norm(B[2]-B[0])

        collision_count = 0

        #################################################################################
        #################################################################################

        trans_mat = self.get_transform_for_AABB(A)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)

        # Case 1: L = Ax
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 2: L = Ay
        if ((A_al[:,1].max() > B_al[:,1].min()) and (A_al[:,1].min() < B_al[:,1].max())):
            collision_count += 1

        # Case 3: L = Az
        if ((A_al[:,2].max() > B_al[:,2].min()) and (A_al[:,2].min() < B_al[:,2].max())):
            collision_count += 1

        ##################################################################################

        trans_mat = self.get_transform_for_AABB(B)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)

        # Case 4: L = Bx
        if ((B_al[:,0].max() > A_al[:,0].min()) and (B_al[:,0].min() < A_al[:,0].max())):
            collision_count += 1

        # Case 5: L = By
        if ((B_al[:,1].max() > A_al[:,1].min()) and (B_al[:,1].min() < A_al[:,1].max())):
            collision_count += 1

        # Case 6: L = Bz
        if ((B_al[:,2].max() > A_al[:,2].min()) and (B_al[:,2].min() < A_al[:,2].max())):
            collision_count += 1

        ##################################################################################

        
        # Case 7: L = Ax X Bx
        trans_mat = self.get_rotation_matrix_for_line(np.cross(Ax,Bx),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 8: L = Ax X By
        trans_mat = self.get_rotation_matrix_for_line(np.cross(Ax,By),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 9: L = Ax X Bz
        trans_mat = self.get_rotation_matrix_for_line(np.cross(Ax,Bz),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 10: L = Ay X Bx
        trans_mat = self.get_rotation_matrix_for_line(np.cross(Ay,Bx),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 11: L = Ay X By
        trans_mat = self.get_rotation_matrix_for_line(np.cross(Ay,By),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 12: L = Ay X Bz
        trans_mat = self.get_rotation_matrix_for_line(np.cross(Ay,Bz),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 13: L = Az X Bx
        trans_mat = self.get_rotation_matrix_for_line(np.cross(Az,Bx),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 14: L = Az X By
        trans_mat = self.get_rotation_matrix_for_line(np.cross(Az,By),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 15: L = Az X Bz
        trans_mat = self.get_rotation_matrix_for_line(np.cross(Az,Bz),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        if collision_count == 15:
            collision = True
        else:
            collision = False

        return collision

    def is_point_in_box(self,box,point):
        """
        Check if proposed grasp will collide with points in cloud.

        Parameters
        ----------
        box: (8,3) ndarray
            Coordinates for vertices of box.
        point: (n,3) ndarray
            Coordinates of points.

        Return
        ------
        collision: (n) ndarray of booleans
            True if the point is within the box.
        """

        AA_transform = self.get_transform_for_AABB(box)
        AABB = self.transform_points(box,AA_transform)
        AA_point = self.transform_points(point,AA_transform)

        x_check = np.logical_and(AA_point[:,0] >= AABB[:,0].min(), AA_point[:,0] <= AABB[:,0].max())
        y_check = np.logical_and(AA_point[:,1] >= AABB[:,1].min(), AA_point[:,1] <= AABB[:,1].max())
        z_check = np.logical_and(AA_point[:,2] >= AABB[:,2].min(), AA_point[:,2] <= AABB[:,2].max())


        return np.logical_and(x_check,np.logical_and(y_check,z_check))



