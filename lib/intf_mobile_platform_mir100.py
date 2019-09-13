import requests
import json
import numpy as np
import time


class MiR_Cart:
    def __init__(self):
        '''
        This function initializes all missions and the actions within those missions.
        To make the robot perform an action, the action must be added to a mission and that
        mission must be queued. Most of these functions modify these actions with the given
        parameters then execute that mission. Actions of a given type can only be replaced
        with another action of the same type. For this reason, there is a dedicated mission 
        for each action type.
        '''

        self.headers = {"Accept-Language": "en_US",
                        "Authorization": "Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="}

        # Initialize mission IDs
        self.mission_id_localize = "000000000000000000000000000000000001"
        self.mission_id_dock = "000000000000000000000000000000000002"
        self.mission_id_lighting = "000000000000000000000000000000000003"
        self.mission_id_moveToPosition = "000000000000000000000000000000000004"
        self.mission_id_relativeMove = "000000000000000000000000000000000005"

        # Initialize action IDs
        self.action_id_localize = "000000000000000000000000000000000006"
        self.action_id_dock = "000000000000000000000000000000000007"
        self.action_id_lighting_type = "000000000000000000000000000000000008"
        self.action_id_lighting_duration = "000000000000000000000000000000000009"
        self.action_id_moveToPosition = "000000000000000000000000000000000010"
        self.action_id_relativeMove = "000000000000000000000000000000000011"

        # JSON messsages to clear previous mission errors and ready the robot for new missions
        self.clear_error = {"clear_error": True}
        self.robot_ready = {"state_id": 3}

        url_0 = "http://mir.com/api/v2.0.0/missions"
        url_1 = "http://"+self.mobile_platform_ip + \
            "/api/v2.0.0/missions/%s/actions" % (self.mission_id_localize)
        url_2 = "http://"+self.mobile_platform_ip + \
            "/api/v2.0.0/missions/%s/actions" % (self.mission_id_dock)
        url_3 = "http://"+self.mobile_platform_ip + \
            "/api/v2.0.0/missions/%s/actions" % (self.mission_id_lighting)
        url_4 = "http://"+self.mobile_platform_ip + \
            "/api/v2.0.0/missions/%s/actions" % (
                self.mission_id_moveToPosition)
        url_5 = "http://"+self.mobile_platform_ip + \
            "/api/v2.0.0/missions/%s/actions" % (self.mission_id_relativeMove)

        mission_localize = {"guid": self.mission_id_localize,
                            "name": "localize", "group_id": "mirconst-guid-0000-0011-missiongroup"}
        mission_dock = {"guid": self.mission_id_dock, "name": "dock",
                        "group_id": "mirconst-guid-0000-0011-missiongroup"}
        mission_lighting = {"guid": self.mission_id_lighting,
                            "name": "lighting", "group_id": "mirconst-guid-0000-0004-missiongroup"}
        mission_moveToPosition = {"guid": self.mission_id_moveToPosition,
                                  "name": "move_to_coordinate", "group_id": "mirconst-guid-0000-0011-missiongroup"}
        mission_relativeMove = {"guid": self.mission_id_relativeMove,
                                "name": "relative_move", "group_id": "mirconst-guid-0000-0011-missiongroup"}

        action_localize = {"action_type": "adjust_localization", "mission_id": self.mission_id_localize,
                           "priority": 0, "guid": self.action_id_localize, "parameters": [{}]}

        action_dock = {"action_type": "docking", "mission_id": self.mission_id_dock, "priority": 0, "guid": self.action_id_dock,
                       "parameters": [{"id": "marker", "value": "e0bfbdb7-8d69-11e9-9d34-94c691a739e9"}]}

        action_lighting_type = {"action_type": "light", "mission_id": self.mission_id_lighting, "priority": 0, "guid": self.action_id_lighting_type,
                                "parameters": [{"id": "speed", "value": "fast"}, {"id": "light_effect", "value": "chase"},
                                               {"id": "color_1", "value": "#00ff00"}, {
                                                   "id": "color_2", "value": "#0000ff"},
                                               {"id": "intensity", "value": 90}, {"id": "timeout", "value": "00:00:10"}]}

        action_lighting_duration = {"action_type": "wait", "mission_id": self.mission_id_lighting, "priority": 1, "guid": self.action_id_lighting_duration,
                                    "parameters": [{"id": "time", "value": "00:00:10"}]}

        action_moveToPosition = {"action_type": "move_to_position", "mission_id": self.mission_id_moveToPosition, "priority": 0, "guid": self.action_id_moveToPosition,
                                 "parameters": [{"id": "x", "value": 0}, {"id": "y", "value": 0},
                                                {"id": "orientation", "value": 0}, {
                                                    "id": "retries", "value": 5},
                                                {"id": "distance_threshold", "value": 0.1}]}

        action_relativeMove = {"action_type": "relative_move", "mission_id": self.mission_id_relativeMove, "priority": 0, "guid": self.action_id_relativeMove,
                               "parameters": [{"id": "x", "value": 0}, {"id": "y", "value": 0},
                                              {"id": "orientation", "value": 0}, {
                                                  "id": "max_linear_speed", "value": 0.5},
                                              {"id": "max_angular_speed", "value": 1.0}, {"id": "collision_detection", "value": True}]}

        q = requests.post(url_0, headers=self.headers, json=mission_localize)
        q = requests.post(url_0, headers=self.headers, json=mission_dock)
        q = requests.post(url_0, headers=self.headers, json=mission_lighting)
        q = requests.post(url_0, headers=self.headers,
                          json=mission_moveToPosition)
        q = requests.post(url_0, headers=self.headers,
                          json=mission_relativeMove)

        r = requests.post(url_1, headers=self.headers, json=action_localize)
        r = requests.post(url_2, headers=self.headers, json=action_dock)
        r = requests.post(url_3, headers=self.headers,
                          json=action_lighting_type)
        r = requests.post(url_3, headers=self.headers,
                          json=action_lighting_duration)
        r = requests.post(url_4, headers=self.headers,
                          json=action_moveToPosition)
        r = requests.post(url_5, headers=self.headers,
                          json=action_relativeMove)

        return

    def dummy_stop(self):
        return False

    def wait_until_mp_is_finished(self, stop_condition='dummy'):
        if stop_condition == 'dummy':
            stop_condition = self.dummy_stop
        time_start = time.time()
        while(self.get_mp_state() != 'Executing'):
            if time.time()-time_start > 5:
                break
        while(self.get_mp_state() in ['Executing', 'EmergencyStop']):
            if stop_condition():
                # Stop Cart Here
                break
        if self.get_mp_state() != 'Ready':
            raise RuntimeError("Cart Error")

    def mp_relative_move(self, forwards=0, left=0, yaw=0, planning=True):
        '''
        Moves the robot relative to its current position in meters/degrees
        Setting the planning variable to 0 disables path planning (The robot moves slowly in this mode and can be used when unloading from vehicles/leaving docking stations)
        Positive yaw values rotate the robot CCW
        '''

        url_0 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/status"
        url_1 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/missions/%s/actions/%s" % (
            self.mission_id_moveToPosition, self.action_id_moveToPosition)
        url_2 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/missions/%s/actions/%s" % (
            self.mission_id_relativeMove, self.action_id_relativeMove)
        url_3 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/mission_queue"

        q = requests.put(url_0, headers=self.headers, json=self.clear_error)
        q = requests.put(url_0, headers=self.headers, json=self.robot_ready)

        if planning is False:
            relative_move = {"action_type": "relative_move", "mission_id": self.mission_id_relativeMove, "priority": 0, "guid": self.action_id_relativeMove,
                             "parameters": [{"id": "x", "value": forwards}, {"id": "y", "value": left},
                                            {"id": "orientation", "value": yaw}, {
                                                "id": "max_linear_speed", "value": 0.5},
                                            {"id": "max_angular_speed", "value": 1.0}, {"id": "collision_detection", "value": True}]}

            mission_queue = {"mission_id": self.mission_id_relativeMove}

            q = requests.put(url_2, headers=self.headers, json=relative_move)
            q = requests.post(url_3, headers=self.headers, json=mission_queue)

        if planning is True:
            # Convert the relative movement requests into a new coordinate
            q = requests.get(url_0, headers=self.headers)
            status_info = q.json()
            for a in status_info:
                if a == "position":
                    pose = np.array(
                        [status_info[a][u'x'], status_info[a][u'y'], status_info[a][u'orientation']])

            X = forwards * \
                np.cos(np.deg2rad(pose[2])) + left * \
                np.sin(np.deg2rad(pose[2])) + pose[0]
            Y = forwards * \
                np.sin(np.deg2rad(pose[2])) + left * \
                np.cos(np.deg2rad(pose[2])) + pose[1]
            Yaw = yaw + pose[2]

            if Yaw < -180:
                Yaw = Yaw + 360
            if Yaw > 180:
                Yaw = Yaw - 360

            move_to_coordinate = {"action_type": "move_to_position", "mission_id": self.mission_id_moveToPosition, "priority": 0, "guid": self.action_id_moveToPosition,
                                  "parameters": [{"id": "x", "value": X}, {"id": "y", "value": Y},
                                                 {"id": "orientation", "value": Yaw}, {
                                                     "id": "retries", "value": 5},
                                                 {"id": "distance_threshold", "value": 0.1}]}

            mission_queue = {"mission_id": self.mission_id_moveToPosition}

            q = requests.put(url_1, headers=self.headers,
                             json=move_to_coordinate)
            q = requests.post(url_3, headers=self.headers, json=mission_queue)

            '''
            This code can be uncommented if you want to queue multiple waypoints at once or run
            waypoints in a loop (eg. making robot move back and forth between waypoints).
            It blocks new missions from queuing until the current mission has completed 
            which ensures the actions within the misions aren't overwritten.

            '''
            #queue_state = 1
            # while queue_state != "None":
            #    q = requests.get(url_3, headers = headers)
            #    status_info = q.json()
            #    for x in status_info:
            #        if x == "mission_queue_id":
            #            queue_state = str(status_info[x])

        return

    def mp_move_to_coordinate(self, x, y, yaw):
        '''
        Moves the robot to a coordinate in the map        
        '''
        url_0 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/status"
        url_1 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/missions/%s/actions/%s" % (
            self.mission_id_moveToPosition, self.action_id_moveToPosition)
        url_2 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/mission_queue"
        url_3 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/status"

        move_to_coordinate = {"action_type": "move_to_position", "mission_id": self.mission_id_moveToPosition, "priority": 0, "guid": self.action_id_moveToPosition,
                              "parameters": [{"id": "x", "value": x}, {"id": "y", "value": y},
                                             {"id": "orientation", "value": yaw}, {
                                                 "id": "retries", "value": 5},
                                             {"id": "distance_threshold", "value": 0.1}]}

        mission_queue = {"mission_id": self.mission_id_moveToPosition}

        q = requests.put(url_0, headers=self.headers, json=self.clear_error)
        q = requests.put(url_0, headers=self.headers, json=self.robot_ready)
        q = requests.put(url_1, headers=self.headers, json=move_to_coordinate)
        q = requests.post(url_2, headers=self.headers, json=mission_queue)

#         This code can be uncommented if you want to queue multiple waypoints at once or run
#         waypoints in a loop (eg. making robot move back and forth between waypoints).
#         It blocks new missions from queuing until the current mission has completed
#         which ensures the actions within the current mission aren't overwritten.

#         #queue_state = 1
#         #while queue_state != "None":
#         #    q = requests.get(url_3, headers = self.headers)
#         #    status_info = q.json()
#         #    for x in status_info:
#         #        if x == "mission_queue_id":
#         #            queue_state = str(status_info[x])
        self.wait_until_mp_is_finished()
        return

    def mp_move_to_pose(self, pose_mtrx):

        pose_vec = self.pose_mtrx_to_vec(pose_mtrx)
        x = pose_vec[0]
        y = pose_vec[1]
        yaw = np.rad2deg(pose_vec[5])
        self.mp_move_to_coordinate(x, y, yaw)
        return

    def mp_start_mission(self):

        # resumes current mission
        url_0 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/status"
        q = requests.put(url_0, headers=self.headers, json=self.clear_error)
        q = requests.put(url_0, headers=self.headers, json=self.robot_ready)

        return

    def mp_pause_mission(self):

        # pauses current mission
        url_0 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/status"
        robot_ready = {"state_id": 4}
        q = requests.put(url_0, headers=self.headers, json=robot_ready)

        return

    def mp_clear_mission_queue(self):

        # cancels current mission and deletes all queued missions
        url_0 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/mission_queue"
        q = requests.delete(url_0, headers=self.headers)

        return

    def mp_update_localization(self):
        '''
        Updates the robot's position in the map
        Should be used in areas with lots of map features
        '''
        url_0 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/status"
        url_1 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/mission_queue"

        mission_queue = {"mission_id": self.mission_id_localize}

        q = requests.put(url_0, headers=self.headers, json=self.clear_error)
        q = requests.put(url_0, headers=self.headers, json=self.robot_ready)
        q = requests.post(url_1, headers=self.headers, json=mission_queue)

        return

    def mp_dock(self):
        '''
        Sends the robot to the specified L or V shaped docking waypoint
        The docking waypoints must first be set up through the MiR GUI
        '''
        url_0 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/status"
        url_1 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/mission_queue"

        mission_queue = {"mission_id": self.mission_id_dock}

        q = requests.put(url_0, headers=self.headers, json=self.clear_error)
        q = requests.put(url_0, headers=self.headers, json=self.robot_ready)
        q = requests.post(url_1, headers=self.headers, json=mission_queue)

        return

    def set_mp_leds(self, time, speed, intensity, effect, color_1, color_2):
        '''
        Controls the robot lighting

        time_format(str): "00:00:00"
        effect(str): "solid", "blink", "fade", "chase", "wave", "rainbow" 
        speed(str): "fast", "slow"
        intensity(int): 0-100
        color_1(str), color_2(str): Red: "#ff0000", Green: "#00ff00", 
            Blue: "#0000ff", White: "#ffffff", Black: "#000000", Yellow: "#ffff00",
            Magenta: "#ff00ff", Cyan: "#00ffff", Orange: "#ffa500", Pink: "#ffc0cb"

        **Black and Pink aren't very noticeable

        If the "solid" effect is used, only color_1 is used
        If the "rainbow" effect is used, neither provided color is used
        '''
        url_0 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/status"
        url_1 = "http://"+self.mobile_platform_ip + \
            "/api/v2.0.0/missions/%s/actions/%s" % (
                self.mission_id_lighting, self.action_id_lighting_type)
        url_2 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/missions/%s/actions/%s" % (
            self.mission_id_lighting, self.action_id_lighting_duration)
        url_3 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/mission_queue"

        light_type = {"action_type": "light", "mission_id": self.mission_id_lighting, "priority": 0,
                      "parameters": [{"id": "speed", "value": speed}, {"id": "light_effect", "value": effect},
                                     {"id": "color_1", "value": color_1}, {
                                         "id": "color_2", "value": color_2},
                                     {"id": "intensity", "value": 100}]}

        light_time = {"action_type": "wait", "mission_id": self.mission_id_lighting, "priority": 1,
                      "parameters": [{"id": "time", "value": time}]}

        mission_queue = {"mission_id": self.mission_id_lighting}

        q = requests.put(url_0, headers=self.headers, json=self.clear_error)
        q = requests.put(url_0, headers=self.headers, json=self.robot_ready)
        q = requests.put(url_1, headers=self.headers, json=light_type)
        q = requests.put(url_2, headers=self.headers, json=light_time)
        q = requests.post(url_3, headers=self.headers, json=mission_queue)

        return

    def get_mp_pose(self):
        # Shows the robot's current x position, y position, orientation and velocity
        url_0 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/status"
        q = requests.get(url_0, headers=self.headers)

        status_info = q.json()
        pose_vec = np.zeros((6))
        for x in status_info:
            if x == "position" or x == "velocity":
                for y in status_info[x]:
                    print("%s(%s):" % (x, y), round(status_info[x][y], 2))
                    if x == "position":
                        if y == 'x':
                            pose_vec[0] = status_info[x][y]
                        elif y == 'y':
                            pose_vec[1] = status_info[x][y]
                        elif y == 'orientation':
                            pose_vec[5] = np.deg2rad(status_info[x][y])
        return self.pose_vec_to_mtrx(pose_vec)

    def get_mp_status(self):
        # Shows the robot's battery level, mission status and current state (eg. paused, executing)
        url_0 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/status"
        q = requests.get(url_0, headers=self.headers)

        status = ["battery_percentage", "mission_text", "state_text"]
        output_text = ["Battery %", "Mission Status", "Robot Status"]

        status_info = q.json()
        for x in status_info:
            for y in range(len(status)):
                if x == str(status[y]):
                    print(output_text[y], ":", status_info[x])
        return

    def get_mp_state(self):
        # Shows the robot's battery level, mission status and current state (eg. paused, executing)
        url_0 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/status"
        q = requests.get(url_0, headers=self.headers)
        status_info = q.json()
        try:
            rtn = status_info['state_text']
        except:
            rtn = self.get_mp_state()
        return rtn

    def get_mp_waypoints(self):
        url_0 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/positions"
        q = requests.get(url_0, headers=self.headers)
        info = q.json()
        wp_list = []
        for i in info:
            wp_list.append(i['name'])

        return wp_list

    def move_mp_to_waypoint(self, waypoint_name):
        url_0 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/positions"
        q = requests.get(url_0, headers=self.headers)
        info = q.json()
        waypoint_guid = None
        for i in info:
            if i['name'] == waypoint_name:
                waypoint_guid = i['guid']
        if waypoint_guid == None:
            raise Exception('No Waypoint:' + waypoint_name)
        url_0 = "http://"+self.mobile_platform_ip + \
            "/api/v2.0.0/positions/" + waypoint_guid
        q = requests.get(url_0, headers=self.headers)
        info = q.json()
        self.mp_move_to_coordinate(
            info['pos_x'], info['pos_y'], info['orientation'])
        return True
