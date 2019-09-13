import numpy as np
import math
import json
import os
import shutil
from rm_config import rm_config
import cv2
import matplotlib.pyplot as plt


class Features:
    def load_feature_lib(self, file_name=None):
        file_name = rm_config['rmstudio_path']+'lib/jupyter_feature_lib.json'
        with open(file_name, 'r') as fp:
            return json.load(fp)

    def save_feature_lib(self, feature_lib, file_name=None):
        if file_name == None:
            file_name = rm_config['rmstudio_path'] + \
                'lib/jupyter_feature_lib.json'
        with open(file_name, 'w') as fp:
            json.dump(feature_lib, fp)
        return True

    def print_feature_lib(self, feature_lib):
        print('Library Features:')
        for item1 in sorted(feature_lib):
            print(item1+":")
            for item2 in sorted(feature_lib[item1]):
                if item2 == 'capture_process_list':
                    print('      ', item2, ':')
                    for i, item3 in enumerate(feature_lib[item1][item2]):
                        print('      ', '      ', i, ':')
                        for item4 in sorted(item3):
                            print('      ', '      ', item4, ' = ',
                                  feature_lib[item1][item2][i][item4])
                else:
                    print('      ', item2, ' = ', feature_lib[item1][item2])

    def find_feature(self, feature, output=[None], rtn_capture=False):
        """
        Finds features

        Returns
        -------
        List of locations corresponding to all features found that meet criteria specified in feature.
        """
        capture = {}
        capture['grasp_list'] = []

        # Check process list for needed capture data: color_image, cloud, depth_image, etc.
        need_color_image = False
        need_cloud = False
        if 'save_to_file' in output:
            save_to_file = True
        else:
            save_to_file = False

        # List of color image descriptors
        color_image_descriptors = ['center_circle']
        # List of depth image descriptors
        depth_image_descriptors = []
        # List of cloud descriptors
        cloud_descriptors = ['downsample',
                             'dbscan', 'nbscan', 'filter_by_size']

        for item in feature['capture_process_list']:
            if item['descriptor'] in color_image_descriptors:
                need_color_image = True
            if item['descriptor'] in cloud_descriptors:
                need_cloud = True

        # Get capture
        self.set_laser_power(1)  # temp
        self.set_laser_state(0)  # temp
        if need_color_image:
            capture['color_image'] = self.get_color_image()
            if 'c' in output or 'all' in output:
                view_ci = self.CI_Viewer()
                view_ci.add_image(capture['color_image'])
                if save_to_file:
                    view_ci.save_to_file()
                else:
                    print('Capture Color Image Output:')
                    view_ci.show()

        if need_cloud:
            self.set_disparity_shift_dist(feature['view_distance'])
            capture['c'] = self.get_cloud()
            if 'c' in output or 'all' in output:
                view_pc = self.PC_Viewer()
                view_pc.add_cloud(self.downsample_cloud(capture['c']))
                if save_to_file:
                    view_pc.save_to_file()
                else:
                    print('Capture Cloud Output:')
                    view_pc.show()

        # Run process list
        for i, item in enumerate(feature['capture_process_list']):
            # Set step id
            step_id = str(i)

            # Set output
            if step_id in output or 'all' in output:
                output_item = True
            else:
                output_item = False

            # Downsample
            if item['descriptor'] == 'downsample':
                # Set input key
                if 'input_key_1' in item.keys():
                    input_key_1 = item['input_key_1']
                elif i == 0:
                    input_key_1 = 'c'
                else:
                    input_key_1 = str(i-1)

                capture[step_id] = self.downsample_cloud(
                    capture[input_key_1], leaf_size=item['leaf_size'])

            # Segment dbscan
            if item['descriptor'] == 'dbscan':
                # Set input key
                if 'input_key_1' in item.keys():
                    input_key_1 = item['input_key_1']
                elif i == 0:
                    input_key_1 = 'c'
                else:
                    input_key_1 = str(i-1)

                # Set Outputs
                if output_item:
                    dbscan_output = ['all']
                    if save_to_file:
                        dbscan_output.append('save_to_file')
                else:
                    dbscan_output = [None]
                capture[step_id] = self.segment_cloud_dbscan(capture[input_key_1],
                                                             search_radius=item['search_radius'],
                                                             min_samples=item['min_samples'], output=dbscan_output)

            # Segment nbscan
            if item['descriptor'] == 'nbscan':
                # Set input key
                if 'input_key_1' in item.keys():
                    input_key_1 = item['input_key_1']
                elif i == 0:
                    input_key_1 = 'c'
                else:
                    input_key_1 = str(i-1)

                # Set Outputs
                if output_item:
                    nbscan_output = ['final']
                    if save_to_file:
                        nbscan_output.append('save_to_file')
                else:
                    output_nbscan = [None]
                capture[step_id] = self.segment_cloud_nbscan(capture[input_key_1],
                                                             curve_threshold=item['curve_threshold'],
                                                             angle_threshold=item['angle_threshold'],
                                                             k=item['k'], output=nbscan_output)

            # Filter by size
            if item['descriptor'] == 'filter_by_size':
                # Set input key
                if 'input_key_1' in item.keys():
                    input_key_1 = item['input_key_1']
                elif i == 0:
                    input_key_1 = 'c'
                else:
                    input_key_1 = str(i-1)

                # Set Outputs
                if output_item:
                    filter_by_size_output = ['final', 'all']
                    if save_to_file:
                        filter_by_size_output.append('save_to_file')
                else:
                    filter_by_size_output = [None]
                capture[step_id] = self.filter_clouds_size(capture[input_key_1],
                                                           dim_1=item['x_axis'], dim_2=item['y_axis'], dim_3=item['z_axis'],
                                                           cluster_size=item['cluster_size'],
                                                           max_clouds_returned=item['max_clouds_returned'], output=filter_by_size_output)

            # Center circle
            if item['descriptor'] == 'center_circle':
                # Set outputs
                if output_item:
                    center_circle_output = ['all']
                    if save_to_file:
                        center_circle_output.append('save_to_file')
                else:
                    center_circle_output = [None]
                circle_found = self.center_circle(feature['view_distance'], min_dist_px=item['min_dist_px'], param_1=item['param_1'], param_2=item['param_2'],
                                                  min_rad_px=item['min_rad_px'], max_rad_px=item['max_rad_px'], blur=item['blur'],
                                                  search_radius_px=item['search_radius_px'], output=center_circle_output)
                if circle_found:
                    current_tcp_pose = self.get_tcp_pose()
                    circPose = self.translate_pose(current_tcp_pose,
                                                   x=self.tcp_to_ci_cam_pose[0, 3],
                                                   y=self.tcp_to_ci_cam_pose[1, 3],
                                                   z=feature['view_distance'],
                                                   frame='self')
                    # Add to capture grasp list
                    capture['grasp_list'].append([circPose, None])
                else:
                    print('No Circles Found')

            # Remove plane
            if item['descriptor'] == 'remove_plane':
                # Set input key
                if 'input_key_1' in item.keys():
                    input_key_1 = item['input_key_1']
                elif i == 0:
                    input_key_1 = 'c'
                else:
                    input_key_1 = str(i-1)

                capture[step_id] = self.remove_planar_surface(
                    capture[input_key_1], item['plane_tol'], rmv_high=True, rmv_low=False)

            # Sort clouds size
            if item['descriptor'] == 'sort_clouds_size':
                # Set input key
                if 'input_key_1' in item.keys():
                    input_key_1 = item['input_key_1']
                elif i == 0:
                    input_key_1 = 'c'
                else:
                    input_key_1 = str(i-1)

                capture[step_id] = self.sort_clouds_size(
                    capture[input_key_1], large_to_small=item['large_to_small'])

            # Sort clouds height
            if item['descriptor'] == 'sort_clouds_height':
                # Set input key
                if 'input_key_1' in item.keys():
                    input_key_1 = item[input_key_1]
                elif i == 0:
                    input_key_1 = 'c'
                else:
                    input_key_1 = str(i-1)

                capture[step_id] = self.sort_clouds_height(
                    capture[input_key_1], high_to_low=item['high_to_low'])

            # Sort clouds distance
            if item['descriptor'] == 'sort_clouds_distance':
                # Set input key
                if 'input_key_1' in item.keys():
                    input_key_1 = item['input_key_1']
                elif i == 0:
                    input_key_1 = 'c'
                else:
                    input_key_1 = str(i-1)

                capture[step_id] = self.sort_clouds_distance(
                    capture[input_key_1], self.origin_pose(), close_to_far=item['close_to_far'])

            # Find Grasp
            if item['descriptor'] == 'find_grasp':
                # Set input key 1
                if 'input_key_1' in item.keys():
                    input_key_1 = item['input_key_1']
                elif i == 0:
                    input_key_1 = 'c'
                else:
                    input_key_1 = str(i-1)

                # Set input key 2
                if 'input_key_2' in item.keys():
                    input_key_2 = item['input_key_2']
                elif i == 0:
                    input_key_2 = '0'
                else:
                    input_key_2 = '0'

                # Set rotate
                if 'rotate_z' in item.keys():
                    rotate_z = item['rotate_z']
                else:
                    rotate_z = True

                # Set outputs
                if output_item:
                    find_grasp_output = ['final']
                    if save_to_file:
                        find_grasp_output.append('save_to_file')
                else:
                    find_grasp_output = None

                # Check if input cloud is empty
                if len(capture[input_key_1]) == 0:
                    continue

                # Check if input of clouds has poses
                if not isinstance(capture[input_key_1][0], (list, tuple)):
                    pose_list = self.get_cloud_pose(capture[input_key_1])

                    # Zip clouds and poses
                    clouds_and_poses = zip(capture[input_key_1], pose_list)

                else:
                    clouds_and_poses = capture[input_key_1]

                grasp_pose, width, object_pose = self.find_grasp(
                    clouds_and_poses, capture[input_key_2], rotate_z=rotate_z, output=find_grasp_output)

                if grasp_pose is not None:
                    grasp_pose = self.get_base_to_camera_pose(
                        cam='pc').dot(grasp_pose)
                    object_pose = self.get_base_to_camera_pose(
                        cam='pc').dot(object_pose)

                    # Add grasp_pose to capture grasp list
                    capture['grasp_list'].append([grasp_pose, width])

                    # Add object_pose to capture grasp list
                    capture['grasp_list'].append([object_pose, None])

            # Find Pose
            if item['descriptor'] == 'find_pose':
                # Set input key
                if 'input_key_1' in item.keys():
                    input_key_1 = item['input_key_1']
                elif i == 0:
                    input_key_1 = 'c'
                else:
                    input_key_1 = str(i-1)

                # Set rotate
                if 'rotate_z' in item.keys():
                    rotate_z = item['rotate_z']
                else:
                    rotate_z = True

                if len(capture[input_key_1]) >= 1:
                    pose = self.get_cloud_pose(capture[input_key_1][0])
                    if output_item:
                        view = self.PC_Viewer()
                        view.add_cloud(capture[input_key_1][0], colorize=True)
                        view.add_axis(pose)
                        view.show()

                    pose = self.get_base_to_camera_pose(cam='pc').dot(pose)
                    if not rotate_z:
                        pose = self.combine_poses(
                            rot_pose=self.get_tcp_pose(), trans_pose=pose)
                else:
                    pose = None

                # Add to capture grasp list
                capture['grasp_list'].append([pose, None])

        # Return full capture
        if rtn_capture:
            return capture
        # Return capture grasp list
        else:
            return capture['grasp_list']

    def find_grasp(self, clouds_and_poses, cloud_full, rotate_z=True, output=None):
        # Find the first cloud within the grippers width and not on fov edge
        cloud_sel = None
        trans_sel = None
        grasp_pose = None
        for item in clouds_and_poses:
            cloud = item[0]
            pose = item[1]
            if not rotate_z:
                pose = self.combine_poses(
                    rot_pose=self.get_tcp_pose(), trans_pose=pose)
            cloud_sel_width = self.get_cloud_width(cloud, pose)
            if cloud_sel_width < 0.1 and not self.is_cloud_on_fov_edge(item[0], tolerance=20):
                cloud_sel = cloud
                trans_sel = pose
                grasp_pose = self.align_pose_to_tcp(trans_sel, frame='tool')
                grasp_pose = self.shift_pose_to_grasp(
                    cloud_full, grasp_pose, cloud_sel_width, step_size=0.0005, min_clearance=0.004)
                if grasp_pose is not None:
                    break

        # If no clouds are within the grippers width
        if grasp_pose is None:
            return None, None, None

        if output != None and ('final' in output or 'all' in output):
            view = self.PC_Viewer()
            view.add_cloud(cloud_full)
            view.add_cloud(cloud_sel, colorize=True)
            view.add_axis(grasp_pose)
            boxes, fboxes = self.get_gripper_boxes(grasp_pose, cloud_sel_width)
            view.add_gripper_boxes(boxes, fboxes)
            if 'save_to_file' in output:
                view.save_to_file()
            else:
                view.show()
        grasp_width = cloud_sel_width*1.5

        return grasp_pose, grasp_width, trans_sel

    def hough_circles_feature(self, holeFeature, output=False):
        """ Return all the circles that match the description in `featureDict` """

        ir_image = self.get_color_image()
        ir_image = cv2.cvtColor(ir_image.copy(), cv2.COLOR_BGR2GRAY)
        ir_image = cv2.medianBlur(
            ir_image, holeFeature['capture_process_list'][0]['blur'])

        circles = cv2.HoughCircles(ir_image, cv2.HOUGH_GRADIENT, 1,
                                   minDist=holeFeature['capture_process_list'][0]['min_dist_px'],
                                   param1=holeFeature['capture_process_list'][0]['param_1'],
                                   param2=holeFeature['capture_process_list'][0]['param_2'],
                                   minRadius=holeFeature['capture_process_list'][0]['min_rad_px'],
                                   maxRadius=holeFeature['capture_process_list'][0]['max_rad_px'])

        if output:
            # Find center circle
            circles = np.uint16(np.around(circles))
            circleimage = ir_image.copy()
            chosen_circle = None
            for circle in circles[0]:
                # Add circle to image
                cv2.circle(
                    circleimage, (circle[0], circle[1]), circle[2], (255, 255, 255), 5)

            plt.figure(figsize=(20, 20))
            plt.imshow(circleimage, vmin=0, vmax=150, cmap='gist_gray')
