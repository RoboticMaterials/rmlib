import cv2
import open3d
import numpy as np

class Segmentation:
    
    def segment_boxes(self,cloud,obj_dims,leaf_size,search_radius):
        cloud_nt = self.remove_planar_surface(cloud,obj_dims[2]/2.0)

        min_cluster_size = (obj_dims[0]*obj_dims[1]/(7.*leaf_size))*1000.

        if len(cloud_nt) > 0:
            obj_clouds = self.segment_cloud(cloud_nt,min_cluster_size=int(min_cluster_size),search_radius=search_radius)
        else:
            obj_clouds = None

        return obj_clouds

    def segment_rings_outer(self,cloud,depth,ir,ring_dims,leaf_size,tolerance,vertical=False):

        cloud_vg = self.downsample_cloud(cloud,0.01)
        height_est = self.get_distance_estimate(cloud_vg)
        scale = self.get_scale_for_real_to_pixel(height_est)

        ring_IR = int(ring_dims[0]*scale/2.0)
        ring_OR = int(ring_dims[1]*scale/2.0)

        ring_thickness = ring_OR-ring_IR
        ring_tolerance = int(tolerance*scale)

        masks = []
        circles = cv2.HoughCircles(ir,cv2.HOUGH_GRADIENT, 1, 40, param1=15, param2=30, minRadius=ring_OR-ring_tolerance, maxRadius=ring_OR+ring_tolerance)
        try:
            circles = circles.reshape(-1,3)
        except:
            return None

        for circle in circles:
            mask = np.zeros(ir.shape)
            circle[2] = circle[2]-int(ring_thickness/2.0)
            cv2.circle(mask,(circle[0],circle[1]),circle[2],(255,255,255),ring_thickness)
            masks.append(mask)

        if masks is None:
            return None
        ring_objs = []
        for mask in masks:
            ring_obj = self.get_cloud_from_mask(mask,depth,ring_dims[2],vertical=vertical)
            ring_obj_vg = self.downsample_cloud(ring_obj,leaf_size)
            ring_objs.append(ring_obj)        
        if len(ring_objs) == 0:
            ring_objs = None
        return ring_objs
    
    def segment_rings_inner(self,cloud,depth,ir,ring_dims,leaf_size,tolerance,vertical=False):

        cloud_vg = self.downsample_cloud(cloud,0.01)
        height_est = self.get_distance_estimate(cloud_vg)
        scale = self.get_scale_for_real_to_pixel(height_est)

        ring_IR = int(ring_dims[0]*scale/2.0)
        ring_OR = int(ring_dims[1]*scale/2.0)

        ring_thickness = ring_OR-ring_IR
        ring_tolerance = int(tolerance*scale)

        masks = []
        circles = cv2.HoughCircles(ir,cv2.HOUGH_GRADIENT, 1, 40, param1=15, param2=30, minRadius=ring_IR-ring_tolerance, maxRadius=ring_IR+ring_tolerance)
        try:
            circles = circles.reshape(-1,3)
        except:
            return None

        for circle in circles:
            mask = np.zeros(ir.shape)
            circle[2] = circle[2]+int(ring_thickness/2.0)
            cv2.circle(mask,(circle[0],circle[1]),circle[2],(255,255,255),ring_thickness)
            masks.append(mask)

        if masks is None:
            return None
        ring_objs = []
        for mask in masks:
            ring_obj = self.get_cloud_from_mask(mask,depth,ring_dims[2],vertical=vertical)
            ring_obj_vg = self.downsample_cloud(ring_obj,leaf_size)
            ring_objs.append(ring_obj)        
        if len(ring_objs) == 0:
            ring_objs = None
        return ring_objs
    
    def segment_cylinders(self,cloud,depth,ir,cyl_dims,leaf_size,search_radius,tolerance):
        if obj_dims[0] >= 0.015:
            ### Large cylinder ###
            cloud_nt = self.remove_planar_surface(cloud,cyl_dims[0]/3.0)

            pcd = open3d.PointCloud()
            pcd.points = Vector3dVector(cloud_nt)
            open3d.estimate_normals(pcd, KDTreeSearchParamHybrid(
                    radius = 0.01, max_nn = 30))
            open3d.orient_normals_to_align_with_direction(pcd,np.array([0.0,0.0,-1.0]))

            thetas = np.arccos(np.array(pcd.normals)).dot([0,0,1])

            itt = 0
            cyl_trans = None
            angle_tolerance = 0.3
            cyl_objs = []

            upward_cloud = np.array(pcd.points)[np.where(thetas>3.14-angle_tolerance)]
            if len(upward_cloud) > 0:
                up_objs = self.segment_cloud(upward_cloud,min_cluster_size=int(0.1/(2*leaf_size)),search_radius=search_radius)
                if len(up_objs) > 0:
                    up_objs = self.sort_clouds_height(up_objs)
                    for cyl_obj in up_objs:
                        cyl_objs.append(cyl_obj)
                        
        else:
            ### Small cylinder ###
            mask = np.zeros(ir.shape)
            ir = np.round(ir*255.0/ir.max()).astype(np.uint8)

            mask[np.where(ir>60)] = 255

            filtered_cloud = self.get_cloud_from_mask(rsc,mask,depth,cyl_dims[0],vertical=True)
            filtered_cloud_vg = self.downsample_cloud(filtered_cloud,leaf_size)

            cyl_objs = self.segment_cloud(filtered_cloud_vg,leaf_size*2.5,0.05/(5.0*leaf_size))
            
        return cyl_objs