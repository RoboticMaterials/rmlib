import numpy as np
import pandas as pd
from pyntcloud import PyntCloud
import matplotlib.pyplot as plt
import cv2
import warnings

class Viewer:
    def show_cloud(self,cloud):
        """
        Show a point cloud.
        
        Parameters
        ----------
        cloud: [n,3] ndarray
            Point cloud
        """
        axis = [
            {
                "color": "red",
                "vertices": [[0, 0, 0], [10, 0, 0]]
            },
            {
                "color": "green",
                "vertices": [[0, 0, 0], [0, 10, 0]]
            },
            {
                "color": "blue",
                "vertices": [[0, 0, 0], [0, 0, 10]]
            }
        ]

        if cloud.shape[1] == 3:
            color = np.zeros(cloud.shape) + 255
            cloud = np.hstack((cloud,color))

        if cloud.shape[1] == 6:
            cloud_df = pd.DataFrame((cloud), columns=['x','y','z','red','green','blue'])
            cloud_pc = PyntCloud(cloud_df)


        return cloud_pc.plot(backend="pythreejs",initial_point_size=0.001,polylines=axis)
    
    def show_ir(self,ir):
        self.fig,self.ax = plt.subplots(figsize=(20,20))
        self.ax.xaxis.tick_top()
        self.ax.set_yticks(np.arange(0,720,80))
        self.ax.set_xticks(np.arange(0,1280,80))

        self.image = ir
        self.ax.imshow(self.image,vmin=0, vmax=150,cmap='gist_gray');
            
            
    class D_Viewer:
        """Viewer for depth images."""
        
        def __init__(self):
            self.fig,self.ax = plt.subplots(figsize=(20,20))
            self.ax.xaxis.tick_top()
            self.ax.set_yticks(np.arange(0,720,80))
            self.ax.set_xticks(np.arange(0,1280,80))
            
            self.image = np.zeros((1280,720))
        
        def add_image(self,image):
            """
            Adds depth image to the viewer.
            
            Parameters
            ----------
            image: [1280,720] ndarray
                Retrieved using rm.get_depth_image(), this image is passed into the viewer. 
            """
            self.image = image
            
        def grid(self):
            """
            Add a grid to the viewer.
            """
            self.ax.grid(color='w',linestyle='-',linewidth=1)
            
        def normalize(self):
            self.image = np.array(self.image*255./self.image.max()).astype(int)
            
        def add_crosshair(self,pos=[640,360]):
            cv2.line(self.image,(pos[0],pos[1]+100),\
                     (pos[0],pos[1]-100),(255,255,255),thickness=2)
            cv2.line(self.image,(pos[0]+100,pos[1]),\
                     (pos[0]-100,pos[1]),(255,255,255),thickness=2)
            
        def add_circle(self,circle,radius=0,thickness=2):
            if len(circle) == 3:
                thickness = radius
                radius = circle[2]
                
            cv2.circle(self.image,(circle[0],circle[1]),radius,(255,255,255),thickness);
            
        def show(self):
            self.ax.imshow(self.image);
            
    class IR_Viewer:
        
        def __init__(self):
            self.fig,self.ax = plt.subplots(figsize=(20,20))
            self.ax.xaxis.tick_top()
            self.ax.set_yticks(np.arange(0,720,80))
            self.ax.set_xticks(np.arange(0,1280,80))
            
            self.image = np.zeros((1280,720))
        
        def add_image(self,image):
            self.image = image
            
        def grid(self):
            self.ax.grid(color='w',linestyle='-',linewidth=1)
            
        def normalize(self):
            self.image = np.array(self.image*255./self.image.max()).astype(int)
            
        def add_crosshair(self,pos=[640,360]):
            cv2.line(self.image,(pos[0],pos[1]+100),\
                     (pos[0],pos[1]-100),(255,255,255),thickness=2)
            cv2.line(self.image,(pos[0]+100,pos[1]),\
                     (pos[0]-100,pos[1]),(255,255,255),thickness=2)
            
        def add_circle(self,circle,radius=0,thickness=2):
            if len(circle) == 3:
                thickness = radius
                radius = circle[2]
                
            cv2.circle(self.image,(circle[0],circle[1]),radius,(255,255,255),thickness);
            
        def show(self,vmin=0,vmax=150):
            self.ax.imshow(self.image,vmin=0, vmax=150,cmap='gist_gray');
            
    class PC_Viewer:
        """Viewer for point clouds."""

        def __init__(self):
            """
            Create a point cloud viewer object.
            
            Example
            -------
            view = rm.PC_Viewer()
            """
            self.cloud = None
            self.lines = []
            self.add_axis(np.eye(4))
            self.add_empty()
            

        def add_cloud(self,cloud,colorize=False,color='rand'):
            """
            Add point cloud to viewer object.
            
            Parameters
            ----------
            cloud: [n,3] ndarray
                Point cloud.
            colorize: bool (optional)
                Add color to the cloud (white cloud if false).
            color: [3,] list or color
                RGB color (random as default).
            """
            if cloud.shape[1] == 3 and colorize==True:
                clr = np.zeros(cloud.shape)
                if color is None:
                    color = [75, 244, 66]
                elif color is 'rand':
                    color = np.random.randint(0,255,3)
                clr[:] = color
                cloud = np.hstack((cloud,clr))

            elif cloud.shape[1] == 3 and colorize==False:
                clr = np.zeros(cloud.shape) + 255
                cloud = np.hstack((cloud,clr))

            if cloud.shape[1] == 6:
                if type(self.cloud) == type(None):
                    self.cloud = cloud
                else:
                    self.cloud = np.vstack((self.cloud,cloud))
                    

        def add_empty(self):
            cloud = np.array(([0.05,0.05,0.000],[0.000,0.0001,0.000]))
            color = np.zeros(cloud.shape)
            cloud = np.hstack((cloud,color))
            self.cloud = cloud
            

        def add_axis(self,transform):
            """
            Add axis to view object.
            
            Parameters
            ----------
            transform: [4,4] ndarray
                Transformation matrix describing the axis.
            """
            start_point = np.hstack((np.zeros((3,3)), np.ones((3,1))))
            end_point = np.hstack((np.eye(3)*0.025, np.ones((3,1))))

            start_trans = transform.dot(start_point.T)
            end_trans = transform.dot(end_point.T)
            
            axis = [
                {
                    "color": "red",
                    "vertices": [list(start_trans[:3,0]), list(end_trans[:3,0])]
                },
                {
                    "color": "green",
                    "vertices": [list(start_trans[:3,1]), list(end_trans[:3,1])]
                },
                {
                    "color": "blue",
                    "vertices": [list(start_trans[:3,2]), list(end_trans[:3,2])]
                }
            ]
            
            self.lines.extend(axis)
            

        def add_line(self,vector,start=[0,0,0],color="grey"):
            """
            Add line to viewer.
            
            Parameters
            ----------
            vector: [3,] list
                [x, y, z] values of vector.
            start: [3,] list
                [x, y, z] coordinates of starting point
            color: [3,] list or color
                RGB color (grey as default).
            """
            line = {
                "color" : color,
                "vertices" : [list(start), list(start+vector)]
            }
            self.lines.append(line)
            
        def add_normals(self,normals,color='#a5a5a5'):
            for i,normal in enumerate(normals):
                self.add_line(normals[i,:3],self.cloud[i,:3],color)

        def add_box(self,vertices,color='#FFA500'):
            """
            Add box to viewer given vertices.
            
            Parameters
            ----------
            vertices: [8,3] ndarray
                Vertices of the box to be added.\n
                [min_x,min_y,max_z]\n
                [max_x,min_y,max_z]\n
                [min_x,max_y,max_z]\n
                [max_x,max_y,max_z]\n
                [min_x,min_y,min_z]\n
                [max_x,min_y,min_z]\n
                [min_x,max_y,min_z]\n
                [max_x,max_y,min_z]
            color: [3,] list or color
                RGB color.
            """
            box = [
                { "color" : color,
                  "vertices": [list(vertices[0]), list(vertices[1])]},
                { "color" : color,
                  "vertices": [list(vertices[1]), list(vertices[3])]},
                { "color" : color,
                  "vertices": [list(vertices[3]), list(vertices[2])]},
                { "color" : color,
                  "vertices": [list(vertices[2]), list(vertices[0])]},

                { "color" : color,
                  "vertices": [list(vertices[4]), list(vertices[5])]},
                { "color" : color,
                  "vertices": [list(vertices[5]), list(vertices[7])]},
                { "color" : color,
                  "vertices": [list(vertices[7]), list(vertices[6])]},
                { "color" : color,
                  "vertices": [list(vertices[6]), list(vertices[4])]},

                { "color" : color,
                  "vertices": [list(vertices[0]), list(vertices[4])]},
                { "color" : color,
                  "vertices": [list(vertices[1]), list(vertices[5])]},
                { "color" : color,
                  "vertices": [list(vertices[2]), list(vertices[6])]},
                { "color" : color,
                  "vertices": [list(vertices[3]), list(vertices[7])]}
            ]
            self.lines.extend(box)
            

        def add_rect(self,vertices,color='#FFD700'):
            """
            Add rectangle to viewer given vertices.
            
            Parameters
            ----------
            vertices: [8,3] ndarray
                Vertices of the box to be added.\n
                [min_x,min_y,z]\n
                [max_x,min_y,z]\n
                [min_x,max_y,z]\n
                [max_x,max_y,z]
            color: [3,] list or color
                RGB color.
            """
            box = [
                { "color" : color,
                  "vertices" : [list(vertices[0]), list(vertices[1])]},
                { "color" : color,
                  "vertices" : [list(vertices[0]), list(vertices[2])]},
                { "color" : color,
                  "vertices" : [list(vertices[2]), list(vertices[3])]},
                { "color" : color,
                  "vertices" : [list(vertices[3]), list(vertices[1])]}
            ]
            self.lines.extend(box)
            

        def add_gripper_boxes(self,boxes,fboxes=None,gripperColor='#00BFFF',fingerColor='#4169E1'):
            """
            Add boxes that describe gripper (gripper boxes and finger boxes).
            
            Parameters
            ----------
            boxes: [2,[8,4]] list of ndarrays
                Gripper boxes.
            fboxes: [3,[8,4]] list of ndarrays
                Finger boxes.
            gripperColor: [3,] list or color
                RGB color.
            fingerColor: [3,] list or color
                RGB color.
                
            Example
            -------
            gripper_boxes,finger_boxes = get_gripper_boxes(transformation_matrix)
            view.add_gripper_boxes(gripper_boxes,finger_boxes)
            """
            for vertices in boxes:
                box = [
                { "color" : gripperColor,
                  "vertices" : [list(vertices[0]), list(vertices[1])]},
                { "color" : gripperColor,
                  "vertices" : [list(vertices[1]), list(vertices[3])]},
                { "color" : gripperColor,
                  "vertices" : [list(vertices[3]), list(vertices[2])]},
                { "color" : gripperColor,
                  "vertices" : [list(vertices[2]), list(vertices[0])]},

                { "color" : gripperColor,
                  "vertices" : [list(vertices[4]), list(vertices[5])]},
                { "color" : gripperColor,
                  "vertices" : [list(vertices[5]), list(vertices[7])]},
                { "color" : gripperColor,
                  "vertices" : [list(vertices[7]), list(vertices[6])]},
                { "color" : gripperColor,
                  "vertices" : [list(vertices[6]), list(vertices[4])]},

                { "color" : gripperColor,
                  "vertices" : [list(vertices[0]), list(vertices[4])]},
                { "color" : gripperColor,
                  "vertices" : [list(vertices[1]), list(vertices[5])]},
                { "color" : gripperColor,
                  "vertices" : [list(vertices[2]), list(vertices[6])]},
                { "color" : gripperColor,
                  "vertices" : [list(vertices[3]), list(vertices[7])]}
                ]
                self.lines.extend(box)

            if fboxes is not None:
                for vertices in fboxes:
                    box = [
                    { "color" : fingerColor,
                      "vertices" : [list(vertices[0]), list(vertices[1])]},
                    { "color" : fingerColor,
                      "vertices" : [list(vertices[1]), list(vertices[3])]},
                    { "color" : fingerColor,
                      "vertices" : [list(vertices[3]), list(vertices[2])]},
                    { "color" : fingerColor,
                      "vertices" : [list(vertices[2]), list(vertices[0])]},

                    { "color" : fingerColor,
                      "vertices" : [list(vertices[4]), list(vertices[5])]},
                    { "color" : fingerColor,
                      "vertices" : [list(vertices[5]), list(vertices[7])]},
                    { "color" : fingerColor,
                      "vertices" : [list(vertices[7]), list(vertices[6])]},
                    { "color" : fingerColor,
                      "vertices" : [list(vertices[6]), list(vertices[4])]},

                    { "color" : fingerColor,
                      "vertices" : [list(vertices[0]), list(vertices[4])]},
                    { "color" : fingerColor,
                      "vertices" : [list(vertices[1]), list(vertices[5])]},
                    { "color" : fingerColor,
                      "vertices" : [list(vertices[2]), list(vertices[6])]},
                    { "color" : fingerColor,
                      "vertices" : [list(vertices[3]), list(vertices[7])]}
                    ]
                    self.lines.extend(box)
                    

        def add_rim_boxes(self,boxes,color='#7FFF00'):
            for vertices in boxes:
                box = {
                { "color" : color,
                  "vertices" : [list(vertices[0]), list(vertices[1])]},
                { "color" : color,
                  "vertices" : [list(vertices[1]), list(vertices[3])]},
                { "color" : color,
                  "vertices" : [list(vertices[3]), list(vertices[2])]},
                { "color" : color,
                  "vertices" : [list(vertices[2]), list(vertices[0])]},

                { "color" : color,
                  "vertices" : [list(vertices[4]), list(vertices[5])]},
                { "color" : color,
                  "vertices" : [list(vertices[5]), list(vertices[7])]},
                { "color" : color,
                  "vertices" : [list(vertices[7]), list(vertices[6])]},
                { "color" : color,
                  "vertices" : [list(vertices[6]), list(vertices[4])]},

                { "color" : color,
                  "vertices" : [list(vertices[0]), list(vertices[4])]},
                { "color" : color,
                  "vertices" : [list(vertices[1]), list(vertices[5])]},
                { "color" : color,
                  "vertices" : [list(vertices[2]), list(vertices[6])]},
                { "color" : color,
                  "vertices" : [list(vertices[3]), list(vertices[7])]}
                }
                self.lines.extend(box)
                
        def clear(self):
            """
            Clear the view object.
            """
            self.cloud = None
            self.lines = []
            self.add_empty()
            self.add_axis(np.eye(4))
            

        def show(self,height=750,width=1200):
            """ 
            Display the point cloud 
            
            Parameters
            ----------
            height: int
                Height of viewer.
            width: int
                Width of viewer.
            """
            
            warnings.filterwarnings('ignore')
                        
            cloud_df = pd.DataFrame( ( self.cloud ) , columns = ['x','y','z','red','green','blue'] )
            cloud_pc = PyntCloud( cloud_df )

            plotObj = cloud_pc.plot( initial_point_size = 0.001, backend="pythreejs", polylines=self.lines, height=height, width=width)

            # === VIEWER DEBUG ===

            if 0:
                print( "Using pyntcloud version:" , pyntcloud.__version__ )
                print( "pyntcloud Location:" , pyntcloud.__file__ )
                print( "Cloud Type:" , type( cloud_pc ) , cloud_pc.__class__.__name__ )
                print( "About to return a plot object of type:" , type( plotObj ) , plotObj.__class__.__name__ )
                print( "Return object has the following props:" )
                print( dir( plotObj ) )
                print( "\tiframe" , type( plotObj.iframe ) , plotObj.iframe.__class__.__name__ )
                print( "\theight" , type( plotObj.height ) , plotObj.height.__class__.__name__ )
                print( "\tsrc" , type( plotObj.src ) , plotObj.src.__class__.__name__ )
                print( "\twidth" , type( plotObj.width ) , plotObj.width.__class__.__name__ )
                print( "\tparams" , type( plotObj.params ) , plotObj.params.__class__.__name__ )
                print( "\t\tThere are" , len( plotObj.params ) , "params" )
                for key , val in plotObj.params.items():
                    print( "\t\t" , key , val )
            # ___ END DEBUG ___
            
            warnings.filterwarnings('always')

            return plotObj



