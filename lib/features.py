import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
import time
import json
import itertools

class Features:  
    def load_feature_lib(self, file_name = None):
        if file_name == None: file_name = self.feature_lib_file_name    
        with open(file_name, 'r') as fp:
            return json.load(fp)
        
    def find_feature(self, capture, feature, output=None):
        """
        Finds features

        Returns
        -------
        List of locations corresponding to all features found that met criteria specifide in feature.
        """

        # Run Process List
        for item in feature['capture_process_list']:
            #Downsample
            if item['descriptor'] == 'downsample':  
                capture['cloud_ds'] = self.downsample_cloud(capture['cloud_raw'], leaf_size=item['leaf_size'])

            #Segment dbscan
            if item['descriptor'] == 'dbscan':
                capture['clouds_seg'] = self.segment_cloud_dbscan(capture['cloud_ds'], search_radius=item['search_radius'], min_samples=item['min_samples'], output=None)
                
            #Filter By Size
            if item['descriptor'] == 'filter_by_size':
                capture['pose_list'], capture['clouds_fil'] = self.filter_clouds_size(capture['clouds_seg'], dim_1=item['x_axis'], dim_2=item['y_axis'], dim_3=item['z_axis'], 
                                                                                      cluster_size=item['cluster_size'], max_clouds_returned=item['max_clouds_returned'], output=None)
        return capture['pose_list'], capture['clouds_fil'], capture['cloud_ds'] 

    def filter_clouds_size(self, clouds, dim_1=[0,100], dim_2=[0,100], dim_3=[0,100], cluster_size = [10,10000000], max_clouds_returned=1000, output=None):
        accepted_clouds = []
        feature_pose_list = []
        for test_cloud in clouds:
            if len(accepted_clouds) < max_clouds_returned:
                add_cloud = False
                length = len(test_cloud)
                if length > cluster_size[0] and length < cluster_size[1]: 
                    dims = self.get_cloud_dimensions(test_cloud)
                    if dim_1[0] < dims[0] < dim_1[1]:
                        if dim_2[0] < dims[1] < dim_2[1]: add_cloud = True
                        if dim_3[0] < dims[1] < dim_3[1]: add_cloud = True
                    if dim_2[0] < dims[0] < dim_2[1]:
                        if dim_1[0] < dims[1] < dim_1[1]: add_cloud = True
                        if dim_3[0] < dims[1] < dim_3[1]: add_cloud = True
                    if dim_3[0] < dims[0] < dim_3[1]:
                        if dim_1[0] < dims[1] < dim_1[1]: add_cloud = True
                        if dim_2[0] < dims[1] < dim_2[1]: add_cloud = True
                                 
                if add_cloud:
                    if output == 'all':
                        print('long_axis:',sorted(dims)[0],' short_axis:',sorted(dims)[1])
                    accepted_clouds.append(test_cloud)
                    feature_pose = self.get_cloud_pose(test_cloud, frame='base')
                    feature_pose_list.append(feature_pose)
            else:
                break

        if accepted_clouds is None:
            raise Exception("No clouds passed through filter")

        if output == 'all':
            # View clouds
            print('Filter Clouds Output:')
            view = self.PC_Viewer()
            for i in accepted_clouds:
                view.add_cloud(i,colorize=True)
            for i in feature_pose_list:
                view.add_axis(i)
            view.show(view='base')
        return feature_pose_list, accepted_clouds

    def get_view_dimensions(self, view_distance):
        camera_z_offset = -self.tcp_to_camera_pose[2,3]
        distance_to_camera = view_distance + camera_z_offset
        m_per_p = self.get_pixles_per_meter(distance_to_camera)
        view_height = 720/m_per_p
        view_width = 1280/m_per_p
        return view_width, view_height 


    def generate_views(self, method, pose, view_distance=None, x_dim=None, y_dim=None, dim_tol=0.2, output=None):      
        view_pose_matrix = np.zeros((1,1,1,4,4))
        if method == 'fixed':
            #Generates a fixed view point at the given trans
            view_pose_matrix[0,0,0,:] = pose.copy()
        
        if method == 'single':
            #Generates a single view point above the feature trans
            view_pose = pose.copy()
            view_pose = self.translate_pose(view_pose, x=-self.tcp_to_camera_pose[0,3], y=-self.tcp_to_camera_pose[1,3], z=-view_distance, frame='self')  
            view_pose_matrix[0,0,0,:] = view_pose

        if method == 'cartesian':
            #Generates view points based on the rectangle dimensions and the field of view
            if x_dim is None or y_dim is None:
                raise Exception("Parameters x_dim and y_dim are required for this method") 

            #Get View Dimensions
            view_dims = self.get_view_dimensions(view_distance)

            # Add Tol to Bin Dims
            x_dim = x_dim*(1+dim_tol)
            y_dim = y_dim*(1+dim_tol)

            # Get Number of View Points
            x_points = math.ceil(x_dim/view_dims[0])
            y_points = math.ceil(y_dim/view_dims[1])

            # Calc Spacing
            if x_points > 1: x_space = (x_dim-view_dims[0])/(x_points-1)
            else: x_space = 0
            if y_points > 1: y_space = (y_dim-view_dims[1])/(y_points-1)
            else: y_space = 0

            # Generate Start pose
            start_pose = pose
            if x_points > 1:start_pose = self.translate_pose(start_pose,x=(x_dim-view_dims[0])/2,frame='self')
            if y_points > 1:start_pose = self.translate_pose(start_pose,y=-(y_dim-view_dims[1])/2,frame='self')

            start_pose = self.generate_views('single',start_pose, view_distance=view_distance)[0,0,0,:]

            # Generate pose Matrix
            view_pose_matrix = self.cartesian_pose_mtrx(start_pose, x_points=x_points, x_space=-x_space, y_points=y_points, y_space=y_space, z_points=1, z_space=0.01, output=output)

        if method == 'cylindrical':
            # Generates one to many view points based on the rectangle dimensions and the field of view
            if x_dim is None or y_dim is None:
                raise Exception("Parameters x_dim and y_dim are required for this method")

            #Get View Dimensions
            view_dims = self.get_view_dimensions(view_distance)

            # Add Tol to Bin Dims
            x_dim = x_dim*(1+dim_tol)
            y_dim = y_dim*1

            # Calc FOV
            view_angle = 2*math.degrees(math.atan((x_dim/2)/view_distance))

            # Get Number of View Points
            camera_view_angle = 100
            axis_points = math.ceil(y_dim/view_dims[1])
            theta_points = math.ceil(view_angle/camera_view_angle)

            # Calc Spacing
            if axis_points > 1: axis_space = (y_dim-view_dims[1])/(axis_points-1)
            else: axis_space = 0
            if theta_points > 1: theta_space = (view_angle-camera_view_angle)/(theta_points-1)
            else: theta_space = 0

            # Generate Start pose
            start_pose = self.translate_pose(pose, y=-(y_dim-view_dims[1])/2, z=-view_distance, frame='self')
            start_pose = self.rotate_pose(start_pose,ry=-math.radians((view_angle-camera_view_angle)/2), frame='self')
            # Generate pose Matrix        
            view_pose_matrix = self.cylindircal_pose_mtrx(start_pose, r_0=0, r_points=1, r_space=0, theta_points=theta_points, theta_space=theta_space, z_points=axis_points, z_space=axis_space, 
                                                 frame='tool', cylinder_axis='y', output=output)
            shape = viewposes_matrix.shape
            for z in range(shape[0]):
                for y in range(shape[1]):
                    for x in range(shape[2]):
                        view_pose_matrix[z,y,x,:] = self.translate_pose(view_pose_matrix[z,y,x,:],x=-self.tcp_to_camera_pose[0,3],y=-self.tcp_to_camera_pose[1,3])

        return view_pose_matrix
        #TODO add mehtods: 
        #3) add convex search pattern to get better views of objects

    
    def get_feature_capture(self, feature, view_method = 'fixed', search_location = None, view_tol=0.2, output=None):
        capture = {}
        capture['cloud_raw'] = None
        capture['image_raw'] = None
        capture['depth_raw'] = None
        
        #accept vec search location from rm_wizard
        if(type(search_location)==list and len(search_location) == 6):
            search_location = self.pose_vec_to_mtrx(search_location)
        
        # Set pose
        if type(search_location) == dict:
            pose = self.pose_vec_to_mtrx(search_location['pose'])     
        elif type(search_location) == list:
            pose = search_location
        else:
            pose = self.get_tcp_pose()
            
        # Set outer_dimensions
        if type(search_location) == dict:
            x_dim=search_location['outer_dimensions'][0]
            y_dim=search_location['outer_dimensions'][1]
        else:
            x_dim=feature['outer_dimensions'][0]
            y_dim=feature['outer_dimensions'][1]
        
        # Set view_distance
        view_distance = feature['view_distance']
        
        # Generate Views
        views = self.generate_views(view_method, pose, view_distance=view_distance, x_dim=x_dim, y_dim=y_dim, dim_tol=view_tol)
        cloud_raw = []
        self.set_disparity_shift(feature['disparity_shift'])
        
        #TODO add code to set lambda function to cature cloud, depth image, image
        self.set_laser_state(True)
        if view_method == 'fixed':
            cloud_raw.append(self.transform_points(self.get_point_cloud(), self.get_base_to_camera_pose()))
        else:
            self.move_through_pose_mtrx(views, lambda:cloud_raw.append(self.transform_points(self.get_point_cloud(), self.get_base_to_camera_pose())),
                                      method='123', path='optimal', speed_per=None)
        self.set_laser_state(False)
        cloud_raw = np.vstack(cloud_raw)
        
        if output in ['all']:
            # View clouds
            print('Get Feature Cloud Output:')
            cloud_ds = self.downsample_cloud(cloud_raw, leaf_size=0.003)
            view = self.PC_Viewer()
            view.add_axis(self.get_base_to_camera_pose())
            view.add_axis(self.get_tcp_pose())
            view.add_cloud(cloud_ds)
            view.show(view='base')
            
        if output == 'gui':
#           save locally
            self.capture = {}
            self.capture['cloud_raw'] = cloud_raw
#           save to file
            view = self.PC_Viewer()
            cloud_ds = self.downsample_cloud(cloud_raw, leaf_size=0.003)
            view.add_cloud(cloud_ds)
            view.saveToFile(view='base')
            return True
        else:
            capture['cloud_raw'] = cloud_raw
            return capture

    