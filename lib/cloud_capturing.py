import numpy as np

class Cloud_Capturing:
    def get_view_dimensions(self, view_distance):
        camera_z_offset = -self.tcp_to_camera_pose[2, 3]
        distance_to_camera = view_distance + camera_z_offset
        m_per_p = self.get_pixels_per_meter(distance_to_camera)
        view_height = 720/m_per_p
        view_width = 1280/m_per_p
        return view_width, view_height

    def generate_views(self, method, pose, view_distance=None, x_dim=None, y_dim=None, dim_tol=0.2, output=None):
        view_pose_matrix = np.zeros((1, 1, 1, 4, 4))
        if method == 'fixed':
            # Generates a fixed view point at the given trans
            view_pose_matrix[0, 0, 0, :] = pose.copy()

        if method == 'single':
            # Generates a single view point above the feature pose
            view_pose = pose.copy()
            view_pose = self.translate_pose(
                view_pose, x=-self.tcp_to_pc_cam_pose[0, 3], y=-self.tcp_to_pc_cam_pose[1, 3], z=-view_distance, frame='self')
            view_pose_matrix[0, 0, 0, :] = view_pose

        if method == 'cartesian':
            # Generates view points based on the rectangle dimensions and the field of view
            if x_dim is None or y_dim is None:
                raise Exception(
                    "Parameters x_dim and y_dim are required for this method")

            # Get View Dimensions
            view_dims = self.get_view_dimensions(view_distance)

            # Add Tol to Bin Dims
            x_dim = x_dim*(1+dim_tol)
            y_dim = y_dim*(1+dim_tol)

            # Get Number of View Points
            x_points = math.ceil(x_dim/view_dims[0])
            y_points = math.ceil(y_dim/view_dims[1])

            # Calc Spacing
            if x_points > 1:
                x_range = np.linspace(0, -x_dim+view_dims[0], x_points)
            else:
                x_range = 0
            if y_points > 1:
                y_range = np.linspace(0, y_dim-view_dims[1], y_points)
            else:
                y_range = 0

            # Generate Start pose
            start_pose = pose
            if x_points > 1:
                start_pose = self.translate_pose(
                    start_pose, x=(x_dim-view_dims[0])/2, frame='self')
            if y_points > 1:
                start_pose = self.translate_pose(
                    start_pose, y=-(y_dim-view_dims[1])/2, frame='self')

            start_pose = self.generate_views(
                'single', start_pose, view_distance=view_distance)[0, 0, 0, :]

            # Generate pose Matrix
            view_pose_matrix = self.cartesian_mtrx_of_poses(
                start_pose, x_range, y_range, [0], frame='tool', output=None)

        if method == 'cylindrical':
            # Generates one to many view points based on the rectangle dimensions and the field of view
            if x_dim is None or y_dim is None:
                raise Exception(
                    "Parameters x_dim and y_dim are required for this method")

            # Get View Dimensions
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
            if axis_points > 1:
                axis_range = np.linspace(0, y_dim-view_dims[1], axis_points)
            else:
                axis_range = 0
            if theta_points > 1:
                theta_range = np.linspace(
                    0, view_angle-camera_view_angle, theta_points)
            else:
                theta_range = 0

            # Generate Start pose
            start_pose = self.translate_pose(
                pose, y=-(y_dim-view_dims[1])/2, z=-view_distance, frame='self')
            start_pose = self.rotate_pose(
                start_pose, ry=-math.radians((view_angle-camera_view_angle)/2), frame='self')
            # Generate pose Matrix
            view_pose_matrix = self.cylindircal_mtrx_of_poses(
                start_pose, [0], theta_range, axis_range, frame='tool', cylinder_axis='y', output=None)

            shape = view_pose_matrix.shape
            for z in range(shape[0]):
                for y in range(shape[1]):
                    for x in range(shape[2]):
                        view_pose_matrix[z, y, x, :] = self.translate_pose(
                            view_pose_matrix[z, y, x, :], x=-self.tcp_to_camera_pose[0, 3], y=-self.tcp_to_camera_pose[1, 3])

        return view_pose_matrix
        # TODO add mehtods:
        # 3) add convex search pattern to get better views of objects

    def get_cloud_of_area(self, view_distance, dims, view_method='single', output=None):
        # Captures a cloud from the current tcp based on the area, view distance, and method provided
        capture = {}
        capture['cloud_raw'] = None

        pose = self.get_tcp_pose()

        x_dim = dims[0]
        y_dim = dims[1]

        # Generate Views
        views = self.generate_views(
            view_method, pose, view_distance=view_distance, x_dim=x_dim, y_dim=y_dim, dim_tol=view_tol)
        cloud_raw = []
        self.set_disparity_shift_dist(view_distance)

        # TODO add code to set lambda function to cature cloud, depth image, image
        self.set_laser_state(True)
        self.set_laser_power(1)
        self.move_through_mtrx_of_poses(views, lambda: cloud_raw.append(self.transform_points(self.get_point_cloud(), self.get_base_to_camera_pose())),
                                        method='123', path='optimal', speed_per=None)
        self.set_laser_state(False)
        cloud_raw = np.vstack(cloud_raw)

        if output in ['all']:
            # View clouds
            print('Cloud Output:')
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
