import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
import time
import pickle

class Assembly:    
    def move_camera_over_tcp(self):
        self.movel([-self.camera_offset[0],-self.camera_offset[1],0,0,0,0],frame="tool",speed=0.1)
        
    def move_tcp_under_camera(self):
        self.movel([self.camera_offset[0],self.camera_offset[1],0,0,0,0],frame="tool",speed=0.1)
        
    def generate_view_pose(self,pose,tcp_height):
        view_pose = pose.copy()
        view_pose[0] += -self.camera_offset[0]
        view_pose[1] += -self.camera_offset[1]
        view_pose[2] += tcp_height
        return view_pose
        
    def tune_circle_locator(self, min_hough_dist=50, param1=40, param2=30, image_blur=5, z_dist_to_cir=0.07, cir_dia=0.02, dia_tol=0.01, max_loops=5): 
        camera_z_offset = -self.camera_offset[2]
        total_z_offset = z_dist_to_cir + camera_z_offset
        
        circle_dia_pixles = self.spc_to_pxls(cir_dia, total_z_offset)
        circle_tol = self.spc_to_pxls(dia_tol, total_z_offset)
        minRadius = round(int((circle_dia_pixles-circle_tol)/2))
        if minRadius < 0: minRadius = 0
        maxRadius = round(int((circle_dia_pixles+circle_tol)/2))
        print('search parameters(circle_dia: %d minR: %dpx maxR: %dpx)'% (circle_dia_pixles, minRadius*2, maxRadius*2))
        
        for i in range(max_loops): 
            #Get ir image 
            ir_image = self.get_ir_image()
            ir_image = cv2.cvtColor(ir_image, cv2.COLOR_BGR2GRAY)

            #Blur image to reduce false positives 
            if image_blur > 0:
                ir_image = cv2.medianBlur(ir_image, image_blur)

            #Find circles
            circles = cv2.HoughCircles(ir_image, cv2.HOUGH_GRADIENT, 1, min_hough_dist, param1=param1 , param2=param2, minRadius=minRadius, maxRadius=maxRadius)

            #Move to top of loop if no circle are found
            if circles is None:
                print("no circles found in image")
                continue

            #Find center circle
            circles = np.uint16(np.around(circles))
            circleimage = ir_image.copy()
            chosen_circle = None
            for circle in circles[0]:
                #Add circle to image
                cv2.circle(circleimage,(circle[0],circle[1]),circle[2],(255,255,255),5)
            #Print circle               
            plt.figure(figsize=(20,20))
            plt.imshow(circleimage,vmin=0, vmax=150,cmap='gist_gray')
        return [min_hough_dist, param1, param2, minRadius, maxRadius, image_blur,total_z_offset]
          
    def circle_locator(self, circle_locator_params, max_movement = 0.02, output=False):
        #Function is designed to handle fasle negative but not false positives
        
        image_center = [640, 360]
        max_loops = 10
        center_tol = 3 
        search_radius = self.spc_to_pxls(max_movement, circle_locator_params[6])
        
        success = False
    
        #Loop till circle is centered or max_loops 
        for i in range(max_loops):
            
            #Get ir image 
            ir_image = self.get_ir_image()
            ir_image = cv2.cvtColor(ir_image, cv2.COLOR_BGR2GRAY)

            #Blur image to reduce false positives 
            if circle_locator_params[5] > 0:
                ir_image = cv2.medianBlur(ir_image, circle_locator_params[5])

            #Find circles
            circles = cv2.HoughCircles(ir_image, cv2.HOUGH_GRADIENT, 1, 
                                       circle_locator_params[0], param1=circle_locator_params[1], param2=circle_locator_params[2], minRadius=circle_locator_params[3], maxRadius=circle_locator_params[4])

            #Move to top of loop if no circle are found
            if circles is None:
                if output:
                    print("no circles found in image")
                continue

            #Find center circle
            circles = np.uint16(np.around(circles))
            circleimage = ir_image.copy()
            chosen_circle = None
            for circle in circles[0]:
                #Add circle to image
                cv2.circle(circleimage,(circle[0],circle[1]),circle[2],(255,255,255),5)
                #Find distance to center
                dist = np.linalg.norm( circle[:2] - np.array(image_center) )
                if dist < search_radius:
                    if chosen_circle is None:
                        chosen_circle = circle
                        chosen_circle_dist = dist
                    else:
                        if chosen_circle_dist > dist:
                            chosen_circle = circle
                            chosen_circle_dist = dist
            if chosen_circle is None:
                if output:
                    print("no circles meet criterion")
                continue
            else:
                success = True
                if output:
                    print("circle found")

            #Add crosshair to image
            cv2.line(circleimage,(image_center[0],image_center[1]+100),(image_center[0],image_center[1]-100),
                 (255,255,255),thickness=3)
            cv2.line(circleimage,(image_center[0]+100,image_center[1]), (image_center[0]-100,image_center[1]),
                 (255,255,255),thickness=3)

            #Check if chosen circle dist is less then center tolerance 
            if chosen_circle_dist < center_tol:
                if(output):
                    plt.figure(figsize=(20,20))
                    plt.imshow(circleimage,vmin=0, vmax=150,cmap='gist_gray')
                    print("circle centered!")
                    print('chosen circle: dist from center %dpx, dia %dpx'% (chosen_circle_dist, chosen_circle[2]*2))
                break
            else:
                #Move camera view to center of circle
                [x_dist_px, y_dist_px] = np.subtract(chosen_circle[:2],image_center)
                x_dist_sp = self.pxls_to_spc(x_dist_px, circle_locator_params[6])
                y_dist_sp = self.pxls_to_spc(y_dist_px, circle_locator_params[6])
                self.movel([-x_dist_sp,-y_dist_sp,0,0,0,0],frame="tool",speed=0.2,accel=0.2)
                
                if(output):
                    plt.figure(figsize=(20,20))
                    plt.imshow(circleimage,vmin=0, vmax=150,cmap='gist_gray')
                    
        return success
                    
    def refine_circle_location(self, feature, max__center_movement=0.02, output=False):
        #Move to view point
        start_view_pose = self.pose_trans(feature['pose'],[-self.camera_offset[0],-self.camera_offset[1],-feature['view_distance'],0,0,0])
        self.movej(start_view_pose)
        #Circle locator
        self.circle_locator(circle_locator_params=feature['circle_locator_params'], max_movement = max__center_movement, output=output)
        return self.pose_trans(self.get_tcp_pose(),[self.camera_offset[0],self.camera_offset[1],feature['view_distance'],0,0,0])
    
    def tune_cloud_locator(self, disparity_shift=150, leaf_size=0.001, search_radius=0.0033, min_cluster_size=50, max_cluster_size=10000000):
        self.set_disparity_shift(disparity_shift)
        cloud = self.get_point_cloud()
        cloud_vg = self.downsample_cloud(cloud,leaf_size=leaf_size)
        cloud_vg_nt = self.remove_planar_surface(cloud_vg)

        # Segment objects with spreading segmentation algorithm.
        object_clouds = self.segment_cloud(cloud_vg_nt,search_radius=search_radius, min_cluster_size=min_cluster_size, max_cluster_size=max_cluster_size )
       
        # Sort object clouds by height.
        object_clouds_sorted = self.sort_clouds_height(object_clouds)
       
        # Pick the highest cloud as our object.
        my_object = object_clouds_sorted[0]

        # Find the transformation matrix of this object representing position and orientation.
        # The transformation matrix will be positioned on the top of the object with the x axis
        # alligned with the principal axis of the object.
        object_transform = self.get_object_transform(my_object,vertical=True)
        
        # Initialize a viewer object
        view = self.PC_Viewer()
        view.add_cloud(cloud_vg,colorize=True,color=[100,100,100])
        view.add_cloud(cloud_vg_nt)
        for object_cloud in object_clouds:
            view.add_cloud(object_cloud,colorize=True)
        view.add_axis(object_transform)
        view.show()
        return [disparity_shift, leaf_size, search_radius, min_cluster_size, max_cluster_size]
    
    def refine_cloud_location(self, feature, output=False):
        #Move to view point
        start_view_pose = self.pose_trans(feature['pose'],[-self.camera_offset[0],-self.camera_offset[1],-feature['view_distance'],0,0,0])
        self.movej(start_view_pose)
        
        total_z_offset = feature['view_distance'] + -self.camera_offset[2]
        self.set_disparity_shift(feature["cloud_locator_params"][0])
        cloud = self.get_point_cloud()
        cloud_vg = self.downsample_cloud(cloud, leaf_size=feature["cloud_locator_params"][1])
        cloud_vg_nt = self.remove_planar_surface(cloud_vg)

        #Segment objects with spreading segmentation algorithm.
        object_clouds = self.segment_cloud(cloud_vg_nt,search_radius=feature["cloud_locator_params"][2],
                                           min_cluster_size=feature["cloud_locator_params"][3],max_cluster_size=feature["cloud_locator_params"][4])

        # Sort object clouds by height.
        object_clouds_sorted = self.sort_clouds_height(object_clouds)
        # Pick the highest cloud as our object.
        my_object = object_clouds_sorted[0]

        # Find the transformation matrix of this object representing position and orientation.
        # The transformation matrix will be positioned on the top of the object with the x axis
        # alligned with the principal axis of the object.
        object_transform = self.get_object_transform(my_object,vertical=True)

        camera_transform = self.get_camera_transform()
        global_transform = self.transform_transform(object_transform,camera_transform)

        # Convert the transformation matrix to a TCP pose.
        object_pose = self.convert_transform_to_pose(global_transform)

        if output:
            #Initialize a viewer object
            view = self.PC_Viewer()
            view.add_cloud(cloud_vg,colorize=True,color=[100,100,100])
            view.add_cloud(cloud_vg_nt)
            for object_cloud in object_clouds:
                view.add_cloud(object_cloud,colorize=True)
            view.add_axis(object_transform)
            view.show()
        return object_pose
    
    def pick_part(self, part_location, start_dist, finger_width=0.108, offset=0):
        #Pick part
        self.movej(np.add(part_location,[0,0,offset,0,0,0]),speed_per=0.5)
        self.set_gripper_width(finger_width)
        pick_pose = self.get_tcp_pose()
        pick_pose[2] = part_location[2] - offset
        self.movel(pick_pose, speed_per=0.5)
        self.close_gripper()
        self.movej([0,0,-start_dist,0,0,0],frame="tool")

        
    def insert_part_tilt(self, insert_location, start_dist, tilt_angle=5, part_offset=0, dia=0, touch_force=1, insert_force=2, max_movement=0.1):
        #Move above part
        #TODO change insertion direction to include more than just z
        self.movej(np.add(insert_location,[0,0,start_dist,0,0,0]))
        
        #Calculate x_offset
        x_offset = part_offset*math.sin(math.radians(tilt_angle))
        #Tilt and offset
        self.movel([-(x_offset+dia/4),0,0,0,0,0],speed=0.05,frame="tool") 
        self.movel([0,0,0,0,math.radians(tilt_angle),0],speed=0.05,frame="tool") 
        
        #Move down till contact
        def a():
            if self.get_wrist_force()[2] < -touch_force:
                return 1
            else:
                return 0
        time.sleep(0.5)
        self.bias_wrist_force()
        self.movel(np.add(self.get_tcp_pose(),[0.0,0.0,-max_movement,0.0,0.0,0.0]),speed=0.01,stop_condition=a)
        
        #Rotate back to vertical
        self.movel([(x_offset+dia/4),0,0.002,0,-math.radians(tilt_angle),0],speed=0.05,frame="tool") 

        #Complete insertion
        def a():
            if self.get_wrist_force()[2] < -insert_force:
                return 1
            else:
                return 0
        time.sleep(0.5)
        self.bias_wrist_force()
        self.movel([0.0,0.0,max_movement,0.0,0.0,0.0],speed=0.01,frame="tool",stop_condition=a)
        
    def insert_part_spiral(self, insert_location, start_dist, touch_force=1, drop_force=1, insert_force=2, max_movement=0.1):
        self.movej(np.add(insert_location,[0,0,start_dist,0,0,0]))

        #Move down till contact
        def a():
            if self.get_wrist_force()[2] < -touch_force:
                return 1
            else:
                return 0
        time.sleep(0.5)
        self.bias_wrist_force()
        self.movel(np.add(self.get_tcp_pose(),[0.0,0.0,-max_movement,0.0,0.0,0.0]),speed=0.01,stop_condition=a)
        
        #Spiral
        def b():
            if abs(self.get_wrist_force()[3]) > .9:
                return 1
            elif abs(self.get_wrist_force()[4]) > .9:
                return 1
            elif self.get_wrist_force()[2] > -drop_force: 
                return 1
            else:
                return 0

        self.spiral(15,0,stepSize=0.00001,maxRadius=0.004,speed=.002,accel=0.5,stop_condition=b)

        #Move down till contact
        def a():
            if self.get_wrist_force()[2] < -insert_force:
                return 1
            else:
                return 0
        time.sleep(0.5)
        self.bias_wrist_force()
        self.movel(np.add(self.get_tcp_pose(),[0.0,0.0,-max_movement,0.0,0.0,0.0]),speed=0.01,stop_condition=a)
                                                    
    def load_feature_dictionary(self, file_name):
        with open(file_name, 'rb') as f:
            feature_dictionary = pickle.load(f)
            print("Feature Dictionary Loaded")
            return feature_dictionary


        

        


    