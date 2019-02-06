import pyrealsense2 as rs
import numpy as np
import time

class RealSense:
    def __init__(self):
        self.camera_controller = None
        self.decimation_filter = rs.decimation_filter()
        self.temporal_filter = rs.temporal_filter()
        self.pc = rs.pointcloud()
        self.points = rs.points()
        self.pipe = rs.pipeline()
        self.running = False
        self.start_camera()
                
    def find_device_that_supports_advanced_mode(self):
        DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07"]
        ctx = rs.context()
        ds5_dev = rs.device()
        devices = ctx.query_devices();
        for dev in devices:
            if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
                if dev.supports(rs.camera_info.name):
                    #print("Found device that supports advanced mode:", dev.get_info(rs.camera_info.name))
                    pass
                return dev
        raise Exception("No device that supports advanced mode was found")
    
    def start_camera(self,x_res=1280,y_res=720,frame_rate=30,visual_preset=1):
        """
        Starts connection with camera. 
        
        Parameters
        ----------
        visual_preset: int (1-4)
            The 4 visual presets can be viewer on the realsense viewer.
        laser_on: bool
            Toggles laser projector. In some cases turning the laser off can be helpful with shiny objects. 
        """
                
        dev = self.find_device_that_supports_advanced_mode()
        self.depth_sensor = dev.first_depth_sensor()
        self.camera_controller = rs.rs400_advanced_mode(dev)
        
        depth_table = self.camera_controller.get_depth_table()
        depth_table.depthUnits = 100
        self.camera_controller.set_depth_table(depth_table)
        
        config = rs.config()
        config.enable_stream(rs.stream.depth,x_res,y_res,rs.format.z16,frame_rate)
        config.enable_stream(rs.stream.infrared,x_res,y_res,rs.format.y8,frame_rate)
        self.depth_sensor.set_option(rs.option.visual_preset, visual_preset)
        profile = self.pipe.start(config)
        video_stream_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        intrinsics = video_stream_profile.get_intrinsics()
        self.fx = intrinsics.fx
        self.fy = intrinsics.fy
        self.cx = intrinsics.ppx
        self.cy = intrinsics.ppy
        
        self.set_laser_state(False)
        
        self.running = True  
        
        
    def stop_camera(self):
        """ Disconnect camera """
        self.pipe.stop()
        self.running = False
    
    def restart_camera(self):
        self.pipe.stop()
        self.running = False
        self.pipe.start()
        self.running = True
        
    def set_laser_state(self,state):
        """
        Toggles laser projector. In some cases turning the laser off can be helpful with shiny objects.
        
        Parameters
        ----------
        state: bool
            True for turning the laser on. 
        """
        
        if self.depth_sensor.supports(rs.option.laser_power):
            if state==False:
                self.depth_sensor.set_option(rs.option.laser_power,0.0)
            else:
                self.depth_sensor.set_option(rs.option.laser_power,150.0)
        time.sleep(0.5)
    
    def set_laser_power(self,power):
        """
        Sets the power of the laser. In some cases turning the laser off can be helpful with shiny objects.
        
        Parameters
        ----------
        power: float
            laser power
        """
        
        if self.depth_sensor.supports(rs.option.laser_power):
            self.depth_sensor.set_option(rs.option.laser_power,power)
        time.sleep(0.1)
        
    def set_disparity_shift(self,disparity_value):
        depth_table = self.camera_controller.get_depth_table()
        depth_table.disparityShift = disparity_value
        self.camera_controller.set_depth_table(depth_table) 
        time.sleep(.5)
        
    def set_disparity_shift_dist(self,camera_dist):
        poly = np.poly1d([2634.54285829, -2341.63812835, 617.51408449])
        disp_shift = int(poly(camera_dist))
        print("Disparity Shift: {}".format(disp_shift))
        self.set_disparity_shift(disp_shift)
        time.sleep(.5)      
        return disp_shift
    
    def tune_disparity_shift(self,start_disparity=0,end_disparity=400,disparity_step=25,output='none'):
        """
        Tunes the disparity shift. If disparity shift is not tuned, the cloud will not \
        render properly. This function starts by setting a low disparity and increases the disparity \
        while counting the number of points in the cloud each time. It then returns the disparity \
        at which it found the most points. This process is very slow so minimize its use as much as \
        possible.
        
        Paramters
        ---------
        start_disparity: int
            Disparity value at which to start looking. 
        end_disparity: int
            Disparity value at which to stop looking.
        disparity_step: int
            The step between each value. Increasing step size will increase speed but decrease accuracy.
            
        """
        
        disparity_values = np.arange(int(start_disparity),int(end_disparity),int(disparity_step))
        num_points = []
        for disparity_value in disparity_values:
            self.set_disparity_shift(disparity_value)
            time.sleep(0.5)
            pts = self.get_point_cloud()
            num_points.append(np.where(np.all(np.equal(pts,np.zeros(3)), axis=1)==False)[0].shape[0])

        disp_shift = disparity_values[np.argmax(num_points)]
        if output in ['info','all']:
            print("Disparity Shift: {}".format(disp_shift))
        self.set_disparity_shift(disp_shift)
        time.sleep(.5)
        return disp_shift
        
    def tune_disparity_shift_dist(self,start_disparity=0,end_disparity=400,disparity_steps=4,output='none'):
        poly = np.poly1d([ 37427.48965781, -59592.3536826,   35110.19443012,  -9502.40118298, 1155.86491098])
        disparity_values = np.linspace(start_disparity,end_disparity,disparity_steps,dtype=int)
        num_points = []
        for disparity_value in disparity_values:
            self.set_disparity_shift(disparity_value)
            time.sleep(0.5)
            pts = self.get_point_cloud()
            num_points.append(np.where(np.all(np.equal(pts,np.zeros(3)), axis=1)==False)[0].shape[0])
        tmp_disp_shift = disparity_values[np.argmax(num_points)]
        self.set_disparity_shift(tmp_disp_shift)
        time.sleep(.3)
        cloud_vg = self.downsample_cloud(self.get_point_cloud(),0.007)

        dist_est = self.get_distance_estimate(cloud_vg)
        disp_shift = int(poly(dist_est))
        self.set_disparity_shift(disp_shift)
        if output in ['info','all']:
            print("Estimated a height of {}m".format(round(dist_est-0.0942,3)))
            print("Disparity Shift: {}".format(disp_shift))
        time.sleep(.5)
        return disp_shift
                          

    def set_auto_exposure_on(self,state):      
        self.depth_sensor.set_option(rs.option.enable_auto_exposure, state)
       
    def set_exposure(self,exposure):
        self.depth_sensor.set_option(rs.option.exposure,exposure)
        
    def get_exposure(self):
        #set from 1 to 165000
        return self.depth_sensor.get_option(rs.option.exposure)
    
    def get_snapshot(self):
        self.set_laser_state(True)
        frames = self.pipe.wait_for_frames()
        
        depth = frames.get_depth_frame()
        depth = self.temporal_filter.process(depth)
        depth_data = np.asanyarray(depth.get_data())

        points = self.pc.calculate(depth)
        
        vtx = np.asanyarray(points.get_vertices())
        pts = np.vstack((vtx[:]['f0'],vtx[:]['f1'],vtx[:]['f2'])).T
        cloud = pts[np.where(np.all(np.equal(pts,np.zeros(3)), axis=1)==False)[0]]
        cloud = cloud.dot(np.array([[-1,0,0],[0,-1,0],[0,0,1]]))
        
        self.set_laser_state(False)
        ir_data = self.get_ir_image()
        
        return cloud,depth_data,ir_data
        
    def get_point_cloud(self):
        """
        Retrieves the point cloud from the realsense camera. 
        
        Parameters
        ----------
        post_proccessing: bool
            Enabling post processing will give a smoother and usually a more accurate cloud; however, \
            it will also slightly decrease the speed.
        """

        self.set_laser_state(True)
        frames = self.pipe.wait_for_frames()
        depth = frames.get_depth_frame()
        depth = self.temporal_filter.process(depth)
        
        points = self.pc.calculate(depth)

        vtx = np.asanyarray(points.get_vertices())
        pts = np.vstack((vtx[:]['f0'],vtx[:]['f1'],vtx[:]['f2'])).T
        cloud = pts[np.where(np.all(np.equal(pts,np.zeros(3)), axis=1)==False)[0]]
        cloud = cloud.dot(np.array([[-1,0,0],[0,-1,0],[0,0,1]]))
        self.set_laser_state(False)
        return cloud
    
    def get_depth_image(self):
        self.set_laser_state(True)
        frames = self.pipe.wait_for_frames()
        depth = frames.get_depth_frame()
        depth = self.temporal_filter.process(depth)
        
        depth_data = depth.get_data()
        self.set_laser_state(False)
        return np.asanyarray(depth_data)
        
    def get_ir_image(self):
        frames = self.pipe.wait_for_frames()
        ir = frames.get_infrared_frame()
        ir_data = ir.get_data()
        return np.asanyarray(ir_data)
    
    def get_scale_for_real_to_pixel(self,height,res=(720,1280)):
        half_width = height * (res[0] - self.cy) / self.fy
        scale = res[0]/(2.0*half_width)
        return scale
    
    def convert_depth_image_to_point_cloud(self, depth, height=None):
        rows, cols = depth.shape
        c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
        z = depth / 10000.0
        x = z * (c - self.cx) / self.fx
        y = z * (r - self.cy) / self.fy
        if height:
            z = np.zeros(depth.shape)+height
        pts = np.dstack((x, y, z)).reshape(-1,3)
        cloud = pts[np.where(np.all(np.equal(pts,np.zeros(3)), axis=1)==False)[0]]
        return cloud.dot(np.array([[-1,0,0],[0,-1,0],[0,0,1]]))

    def convert_point_cloud_to_depth_image(self, cloud):
        cloud = cloud.dot(np.linalg.inv(np.array([[-1,0,0],[0,-1,0],[0,0,1]])))
        image = np.zeros((720,1280),dtype=np.uint16)
        x = cloud[:,0]
        y = cloud[:,1]
        z = cloud[:,2]
        
        c = np.floor(x*self.fx/z + self.cx).astype(int)
        r = np.floor(y*self.fy/z + self.cy).astype(int)
        idxs = np.where(np.logical_and(np.logical_and(c>=0, c<1280),np.logical_and(r>=0, r<720)))
        c = c[idxs]
        r = r[idxs]
        z = z[idxs]
        
        z = z*10000.0
        image[r,c] = z
        return image
