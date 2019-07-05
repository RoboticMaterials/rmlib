import numpy as np
from scipy.spatial import Delaunay
import numpy as np
import math
import cv2

class Image_Processing: 
    def pxls_to_spc(self, pxls_dist, z_cam):
        # Convert pixles to space
        return pxls_dist / self.get_pixles_per_meter(z_cam)

    def spc_to_pxls(self, spc_dist, z_cam):
        # Convert space to pixles
        return spc_dist * self.get_pixles_per_meter(z_cam)
    
    def concave_object(self,cloud,ir,alpha=0.1,size_factor=0.4):

        mask = np.zeros((ir.shape[0],ir.shape[1])).astype(np.uint8)
        mask[np.where(self.convert_point_cloud_to_depth_image(cloud)>0)] = 255
        mask = cv2.resize(mask,None,fx=size_factor,fy=size_factor)

        ret,thresh = cv2.threshold(mask, 0, 255, 0)

        im2,contours,hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        contour = np.vstack(contours)

        concave_hull,edge_points = self.alpha_shape(np.squeeze(contour),alpha=alpha)

        raw_dist = np.zeros(mask.shape, dtype=np.float32)-100.
        x_min = concave_hull[:,0,0].min()
        x_max = concave_hull[:,0,0].max()
        y_min = concave_hull[:,0,1].min()
        y_max = concave_hull[:,0,1].max()
        for i in range(x_min,x_max):
            for j in range(y_min,y_max):
                raw_dist[j,i] = cv2.pointPolygonTest(concave_hull, (i,j), True)

        inside_idxs = np.where(raw_dist >= 1.)
        ir = cv2.resize(ir,None,fx=size_factor,fy=size_factor)
        img = np.zeros((mask.shape[0],mask.shape[1],3)).astype(np.uint32)
        img[:,:,1] = 255
        img[inside_idxs] = ir[inside_idxs]
        
#         img = cv2.resize(img,None,fx=1./size_factor,fy=1./size_factor)

        return img

    def alpha_shape(self,points, alpha):
        """
        Compute the alpha shape (concave hull) of a set of points.

        Parameters
        ----------
        points: Iterable container of points.
            Points to be used to find the alpha shape.
        alpha: 
            alpha value to influence the smoothness of the border. Smaller numbers \
            don't fall inward as much as larger numbers. Too large, and you lose information.
            
        Reference
        ---------
        http://blog.thehumangeo.com/2014/05/12/drawing-boundaries-in-python/
        
        """
        if len(points) < 4:
            # When you have a triangle, there is no sense
            # in computing an alpha shape.
            return geometry.MultiPoint(list(points)).convex_hull

        def add_edge(edges, edge_points, coords, i, j):
            """
            Add a line between the i-th and j-th points,
            if not in the list already
            """
            if (i, j) in edges or (j, i) in edges:
                # already added
                return
            edges.add( (i, j) )
            edge_points.append(coords[ [i, j] ])

        coords = np.array(points)
        tri = Delaunay(coords)
        edges = set()
        edge_points = []
        # loop over triangles:
        # ia, ib, ic = indices of corner points of the
        # triangle
        for ia, ib, ic in tri.vertices:
            pa = coords[ia]
            pb = coords[ib]
            pc = coords[ic]
            # Lengths of sides of triangle
            a = math.sqrt((pa[0]-pb[0])**2 + (pa[1]-pb[1])**2)
            b = math.sqrt((pb[0]-pc[0])**2 + (pb[1]-pc[1])**2)
            c = math.sqrt((pc[0]-pa[0])**2 + (pc[1]-pa[1])**2)
            # Semiperimeter of triangle
            s = (a + b + c)/2.0
            # Area of triangle by Heron's formula
            area = math.sqrt(s*(s-a)*(s-b)*(s-c))
            circum_r = a*b*c/(4.0*area)
            # Here's the radius filter.
            if circum_r < 1.0/alpha:
                add_edge(edges, edge_points, coords, ia, ib)
                add_edge(edges, edge_points, coords, ib, ic)
                add_edge(edges, edge_points, coords, ic, ia)
        m = geometry.MultiLineString(edge_points)
        triangles = list(polygonize(m))
        return np.array(cascaded_union(triangles).buffer(1).exterior.coords).reshape(-1,1,2).astype(int), np.array(edge_points)