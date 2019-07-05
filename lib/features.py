import numpy as np
import math
import json
import os
import shutil

class Features:    
    def load_feature_lib(self, file_name = None):
        if file_name == None: file_name = self.feature_lib_file_name    
        with open(file_name, 'r') as fp:
            return json.load(fp)
    
    def save_feature_lib(self, feature_lib, file_name = None):
        if file_name == None: file_name = self.feature_lib_file_name 
        with open(file_name, 'w') as fp:
            json.dump(feature_lib, fp)
        return True
    
    def print_feature_lib(self, feature_lib):
        print('Library Features:')
        for item1 in sorted(feature_lib):
            print(item1+":")
            for item2 in sorted(feature_lib[item1]):
                if item2 == 'capture_process_list':
                    print('      ',item2,':')
                    for i, item3 in enumerate(feature_lib[item1][item2]):
                        print('      ','      ',i,':')
                        for item4 in sorted(item3):
                            print('      ','      ',item4,' = ',feature_lib[item1][item2][i][item4])
                else: 
                    print('      ',item2,' = ',feature_lib[item1][item2])
    
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
                
            #Segment nbscan
            if item['descriptor'] == 'nbscan':
                capture['clouds_seg'] = self.segment_cloud_nbscan(capture['cloud_ds'], curve_threshold=item['curve_threshold'], angle_threshold=item['angle_threshold'], k = item['k'], output=None)
                
            #Filter By Size
            if item['descriptor'] == 'filter_by_size':
                print(0)
                capture['clouds_fil'] = self.filter_clouds_size(capture['clouds_seg'], dim_1=item['x_axis'], dim_2=item['y_axis'], dim_3=item['z_axis'], 
                                                                                      cluster_size=item['cluster_size'], max_clouds_returned=item['max_clouds_returned'], output=None)
                print(1)
                capture['pose_list'] = self.get_cloud_pose(capture['clouds_fil'])
                print(2)
                
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
            view.show(view='base')
            
        return accepted_clouds



    def get_view_dimensions(self, view_distance):
        camera_z_offset = -self.tcp_to_camera_pose[2,3]
        distance_to_camera = view_distance + camera_z_offset
        m_per_p = self.get_pixels_per_meter(distance_to_camera)
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
            if x_points > 1: x_range = np.linspace(0, -x_dim+view_dims[0], x_points)
            else: x_range = 0
            if y_points > 1: y_range = np.linspace(0, y_dim-view_dims[1], y_points)
            else: y_range = 0
                

            # Generate Start pose
            start_pose = pose
            if x_points > 1:start_pose = self.translate_pose(start_pose,x=(x_dim-view_dims[0])/2,frame='self')
            if y_points > 1:start_pose = self.translate_pose(start_pose,y=-(y_dim-view_dims[1])/2,frame='self')

            start_pose = self.generate_views('single',start_pose, view_distance=view_distance)[0,0,0,:]

            # Generate pose Matrix   
            view_pose_matrix = self.cartesian_mtrx_of_poses(start_pose, x_range, y_range, [0], frame='tool', output=None)

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
            if axis_points > 1: axis_range = np.linspace(0,y_dim-view_dims[1],axis_points) 
            else: axis_range = 0
            if theta_points > 1: theta_range = np.linspace(0,view_angle-camera_view_angle,theta_points)
            else: theta_range = 0

            # Generate Start pose
            start_pose = self.translate_pose(pose, y=-(y_dim-view_dims[1])/2, z=-view_distance, frame='self')
            start_pose = self.rotate_pose(start_pose,ry=-math.radians((view_angle-camera_view_angle)/2), frame='self')
            # Generate pose Matrix        
            view_pose_matrix = self.cylindircal_mtrx_of_poses(start_pose, [0], theta_range, axis_range, frame='tool', cylinder_axis='y', output=None)
        
        
            shape = view_pose_matrix.shape
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
            self.move_through_mtrx_of_poses(views, lambda:cloud_raw.append(self.transform_points(self.get_point_cloud(), self.get_base_to_camera_pose())),
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
            
    def wizard_segment_cloud(self,feature, cloud_name):
        self.capture['cloud_ds'] = self.downsample_cloud(self.capture[cloud_name], leaf_size=feature['capture_process_list'][0]['leaf_size'])
        if feature['capture_process_list'][1]['descriptor'] == 'dbscan':      
            self.capture['clouds_seg'] = self.segment_cloud_dbscan(self.capture['cloud_ds'], 
                                                                   search_radius=feature['capture_process_list'][1]['search_radius'], 
                                                                   min_samples=feature['capture_process_list'][1]['min_samples'], 
                                                                   output=None)
             
        if feature['capture_process_list'][1]['descriptor'] == 'nbscan':
            self.capture['clouds_seg'] = self.segment_cloud_nbscan(self.capture['cloud_ds'], 
                                                                   curve_threshold=feature['capture_process_list'][1]['curve_threshold'], 
                                                                   angle_threshold=feature['capture_process_list'][1]['angle_threshold'],
                                                                   k = feature['capture_process_list'][1]['k'], 
                                                                   output=None)  
        view = self.PC_Viewer()
        view.add_cloud(self.capture['cloud_ds'])
        for i in self.capture['clouds_seg']:
            view.add_cloud(i,colorize=True)
        view.saveToFile(view='base')
        return True
    
    def wizard_filter_clouds_size(self,feature):
        print(1)
        accepted_clouds = self.filter_clouds_size(self.capture['clouds_seg'], 
                                                                     dim_1=feature['capture_process_list'][2]['x_axis'], 
                                                                     dim_2=feature['capture_process_list'][2]['y_axis'], 
                                                                     dim_3=feature['capture_process_list'][2]['z_axis'], 
                                                                     cluster_size=feature['capture_process_list'][2]['cluster_size'], 
                                                                     max_clouds_returned=feature['capture_process_list'][2]['max_clouds_returned'], 
                                                                     output=None)
        object_pose_list = self.get_cloud_pose(accepted_clouds)
        view = self.PC_Viewer()
        view.add_cloud(self.capture['cloud_ds'])
        for i in accepted_clouds:
            view.add_cloud(i,colorize=True)
        for i in object_pose_list:
            view.add_axis(i)
        view.saveToFile(view='base')
        return True 
    
    def wizard_set_view_cloud(self, cloud_name):
        cloud = self.downsample_cloud(self.capture[cloud_name],leaf_size=0.0025)
        view = self.PC_Viewer()
        view.add_cloud(cloud)
        view.saveToFile(view='base')
        return True
        
    
    def wizard_remove_planar_surface(self, rmv_plane_tol):
        self.capture['cloud_pr']  = self.remove_planar_surface(self.capture['cloud_raw'],rmv_tolerance=rmv_plane_tol, rmv_high=False, rmv_low=True)
        self.wizard_set_view_cloud('cloud_pr')
        return True
    
    def wizard_remove_container(self, container):
        self.capture['cloud_cr']  = self.crop_to_inner_dims(self.capture['cloud_raw'],container)
        self.wizard_set_view_cloud('cloud_cr')
        return True
        
    def update_point_cloud(self):
        self.set_disparity_shift(100)
        cloud = self.get_point_cloud()
        cloud = self.downsample_cloud(cloud,leaf_size=0.0025)

        clr = np.zeros(cloud.shape)
        color = np.random.randint(0,255,3)
        clr[:] = color
        cloud = np.hstack((cloud,clr))
                
#         clr = np.zeros(cloud.shape) + 255
#         cloud = np.hstack((cloud,clr))

        cloud_df = pd.DataFrame((cloud), columns = ['x','y','z','red','green','blue'])

        cloud_pc = PyntCloud(cloud_df)
        cloud_pc.to_file("/home/nvidia/dev_rmstudio/lib/html/feature_wizard/models/point_cloud.ply")
        return True
    
