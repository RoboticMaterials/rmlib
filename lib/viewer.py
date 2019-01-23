import numpy as np
import pandas as pd
from pyntcloud import PyntCloud
import matplotlib.pyplot as plt
import cv2

class Viewer:
    def show_cloud(self,cloud):
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
        
    class DI_Viewer:
        
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
        """ Display point clouds """

        def __init__(self):
            self.cloud = None
            self.lines = []
            self.add_axis(np.eye(4))
            

        def add_cloud(self,cloud,colorize=False,color='rand'):
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
            

        def add_line(self,vector,color="black"):
            line = {
                "color" : color,
                "vertices" : [list(vector[0]), list(vector[1])]
            }
            self.lines.append(line)
            

        def add_box(self,vertices,color='#FFA500'):
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
            self.cloud = None
            self.lines = []
            self.add_empty()
            self.add_axis(np.eye(4))
            

        def show(self):
            """ Display the point cloud """
            cloud_df = pd.DataFrame( ( self.cloud ) , columns = ['x','y','z','red','green','blue'] )
            cloud_pc = PyntCloud( cloud_df )

#             plotObj = cloud_pc.plot( initial_point_size = 0.001, backend="pythreejs")
            plotObj = cloud_pc.plot( initial_point_size = 0.001, backend="pythreejs", polylines=self.lines)

            # scene = pythreejs.Scene(children=children) , 
                # https://github.com/daavoo/pyntcloud/blob/master/pyntcloud/plot/pythreejs_backend.py

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

            return plotObj



