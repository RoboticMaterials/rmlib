class URCap: 
    def get_urcap_feature_list(self):
        print('a')
        try:
            feature_lib = self.load_feature_lib(self.feature_lib_file_name)
            string = ''
            for i in feature_lib:
                string = string + i + '%'
            string=string[:-1]
            print("Object definitions requested")
            print("b")
            return string
        except Exception as e: 
            print(e)
            return "generic"

    def get_urcap_feature_pose(self, feature_name, remove_plane=True):
        plane_tol = 0.0045
        feature_lib = self.load_feature_lib(self.feature_lib_file_name)

        #Get Cloud
        self.set_disparity_shift(100)
        capture = self.get_feature_capture(feature_lib[feature_name])

        #Downsample Cloud
        cloud_full_ds = self.downsample_cloud(capture['cloud_raw'],leaf_size=0.003)

        #Remove Plane
        if remove_plane:
            capture['cloud_raw'] = self.remove_planar_surface(capture['cloud_raw'], plane_tol, rmv_high = False, rmv_low = True)

        #Find Feature
        trans, clouds_filt, cloud_ds = self.find_feature(capture, feature_lib[feature_name])

        #Zip clouds and trans
        cld_and_trans = zip(clouds_filt,trans)

        #Sort largest to smallest
        cld_and_trans = sorted(cld_and_trans,key=lambda x: len(x[0]))
        cld_and_trans.reverse()

        #Find the first cloud within the grippers width
        cloud_sel = None
        trans_sel = None
        for item in cld_and_trans:
            cloud_sel_width = self.get_cloud_width(item[0],item[1])
            if cloud_sel_width < 0.1:
                cloud_sel = item[0]
                trans_sel = item[1]
                grasp_pose = self.align_pose_to_tcp(trans_sel)
                grasp_pose = self.shift_pose_to_grasp(cloud_full_ds,grasp_pose,cloud_sel_width,step_size=0.001)
                if grasp_pose is not None:
                    break
                    
        pose = self.pose_mtrx_to_vec(grasp_pose)
        
        # Set Approach
        z_offset = self.get_tcp_pose_vec()[2]
        approach_pose = self.translate_pose(grasp_pose, z=-z_offset, frame='self')
        
        width = (cloud_sel_width*1.5).tolist()
        
        success = 1
        
        pose.extend(self.pose_mtrx_to_vec(approach_pose))
        pose.append(width)
        pose.append(success)
        
        print(pose)
        return pose



