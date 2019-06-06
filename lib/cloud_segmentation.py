import cv2
import open3d
import numpy as np
from sklearn.cluster import DBSCAN
from pyntcloud import PyntCloud

class Cloud_Segmentation:
    
    def segment_cloud_dbscan(self, cloud, search_radius=0.005, min_samples=4, output=None):
        """
        Segments cloud via DBSCAN. DBSCAN is a spreading algorithm that looks for points within radius to add to segmented cloud\
        and subsequently uses that new point to spread.

        Parameters
        ---------
        cloud: [n,3] ndarray
            Point cloud.
        search_radius: float (optional)
            Radius that DBSCAN uses to search around current seed point.
        min_cluster_size: int (optional)
            Minimum amounts of points in a region to be considered an object.

        Returns
        -------
        object_clouds: list of m [k,3] ndarrays
            A list of point cloud objects.
        """

        db = DBSCAN(eps=search_radius,min_samples=min_samples).fit(cloud)

        objects_ids = np.unique(db.labels_)
        num_objects = objects_ids.shape[0]

        if objects_ids[0] == -1:
            cloud = cloud[np.where(db.labels_ != -1)]
            db.labels_ = db.labels_[np.where(db.labels_ != -1)] 

        objects_clouds = []
        for i in objects_ids:
            object_idxs = np.where(db.labels_ == i)
            if len(object_idxs[0]) < 1:
                cloud = cloud[np.where(db.labels_ != i)]
                db.labels_ = db.labels_[np.where(db.labels_ != i)]
            else:
                objects_clouds.append(cloud[object_idxs])
        
        if output == 'all':
            # View clouds
            print('Segment Cloud dbscan Output:')
            view = self.PC_Viewer()
            view.add_cloud(cloud,colorize=True,color=[100,100,100])
            for cloud in objects_clouds:
                view.add_cloud(cloud,colorize=True)
            view.show()
            
        return objects_clouds

