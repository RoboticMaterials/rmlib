# Functions that the URCAPs use to communicate with RMStudio

from rm_config import rm_config

import time  # temp


class URCap:
    def get_urcap_feature_list(self):
        """ Return feature information as a string """
        try:
            feature_lib = self.load_feature_lib()
            string = ''
            for i in feature_lib:
                string = string + i + '%'
            string = string[:-1]
            print("Object definitions requested")
            return string
        except Exception as e:
            print(e)
            return "error"

    def get_urcap_feature_pose(self, feature_name, remove_plane=True):
        """ Capture feature data, then determine the feature pose """
        # Get feature info from the lib
        feature_lib = self.load_feature_lib()
        if feature_name not in feature_lib:
            raise KeyError("get_urcap_feature_pose: Feature name",
                           feature_name, "is not in the library")

        # Find feature
        grasp = self.find_feature(feature_lib[feature_name], output=[
                                  '5', 'save_to_file'], rtn_capture=False)

        # If no grasps are found
        if len(grasp) == 0 or None in grasp:
            return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        info_list = self.pose_mtrx_to_vec(grasp[0][0])

        # Set Approach
        z_offset = self.get_tcp_pose_vec()[2]
        approach_pose = self.translate_pose(
            grasp[0][0], z=-z_offset, frame='self')

        success = 1
        info_list.extend(self.pose_mtrx_to_vec(approach_pose))
        info_list.append(grasp[0][1].tolist())
        info_list.append(success)

        return info_list

    def urcap_return_mp_waypoints(self):
        return self.urcap_mp_waypoints

    def urcap_get_mp_waypoints(self):
        if rm_config['import']['mobile_platform']:
            waypoints_list = self.get_mp_waypoints()
            waypoints_string = waypoints_list[1]
            for waypoint in waypoints_list[2:]:
                waypoints_string = waypoints_string + '%' + waypoint
            self.urcap_mp_waypoints = waypoints_string
        else:
            self.urcap_mp_waypoints = 'no_cart_connected'

        return True
