#!/usr/bin/env python
# coding: utf-8

# In[1]:


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Circle
from IPython.display import display, clear_output
import numpy as np
import cv2
import operator


# In[2]:


import Utils_3D as Ut3D


# In[ ]:


def linkDirection(orientation, link_length):
    
    if (orientation == (0,0,0)):
        link_array = np.array([0,0,link_length])
    
    elif (orientation == (90,0,0)):
        link_array = np.array([0,-link_length, 0])
    
    elif (orientation == (-90,0,0)):
        link_array = np.array([0,link_length, 0])
    
    elif (orientation == (0,90,0)):
        link_array = np.array([link_length,0,0])
    
    elif (orientation == (0,-90,0)):
        link_array = np.array([-link_length,0,0])
    
    elif (orientation == (0,0,90)):
        link_array = np.array([0,0,link_length])
    
    elif (orientation == (0,0,-90)):
        link_array = np.array([0,0,link_length])
    
    else:
        print("Wrong Orientation")
        mov_angle = None
    
    return link_array


# In[ ]:


def mov_dir(orientation, angle):
    
    if (orientation == (0,0,0)):
        mov_angle = (0,0,angle)
    
    elif (orientation == (90,0,0)):
        mov_angle = (0,-angle,0)
    
    elif (orientation == (-90,0,0)):
        mov_angle = (0,angle,0)
    
    elif (orientation == (0,90,0)):
        mov_angle = (angle,0,0)
    
    elif (orientation == (0,-90,0)):
        mov_angle = (-angle,0,0)
    
    elif (orientation == (0,0,90)):
        mov_angle = (0,0,angle)
    
    elif (orientation == (0,0,-90)):
        mov_angle = (0,0,angle)
    
    else:
        print("Wrong Orientation")
        mov_angle = None
    
    return mov_angle


# In[47]:


def drawLink(ax, x1, y1, z1, link_length, joint_type, orientation, movement_angle):
    
    LinkArray = linkDirection(orientation, link_length)
            
    RX, RY, RZ = movement_angle
    x2, y2, z2 = Ut3D.Rot(RX, RY, RZ).dot(LinkArray) + np.array([x1, y1, z1])
    
    origin = np.array([x1, y1, z1])
    vector = np.array([x2 - x1, y2 - y1, z2 - z1])
    
    X, Y, Z, U, V, W = Ut3D.vec_to_quiver(origin, vector)
    ax.quiver(X, Y, Z, U, V, W, arrow_length_ratio = 0.05, color='k')
    
    if (joint_type == 'Prismatic Joint'):
        ax.quiver(X, Y, Z, U, V, W, arrow_length_ratio = 0.1, color='y')
        ax.quiver(X+1, Y+1, Z+1, U, V, W, arrow_length_ratio = 0.1, color='y')
    
    return x2, y2, z2


# In[6]:


def kinematics_diagram(ax, frame_list):
    
    ############################################################################################################
    # Frame One:
    
    frameOne = frame_list[0]
    
    orientation = frameOne[3]
    movement_angle = mov_dir(orientation, frameOne[4])
    
    x, y, z = frameOne[0], frameOne[1], frameOne[2]
    
    if frameOne[5] == 'Revolute Joint':
        
        radius = 10
        height = 20
        
        Ut3D.drawCylinder(ax, x, y, z, radius, height, orientation, movement_angle)
        Ut3D.drawFrameAxis(ax, x, y, z, orientation, movement_angle)
        
    elif frameOne[5] == 'Prismatic Joint':
        
        L = 20
        
        Ut3D.drawCube(ax, x, y, z, L, movement_angle)
        Ut3D.drawFrameAxis(ax, x, y, z, orientation, movement_angle)
        
    else:
        print('Wrong Joint Type Defined.')
        
    link_length = frameOne[6]
    x, y, z = drawLink(ax, x, y, z, link_length, frameOne[5], orientation, movement_angle)
        
    ############################################################################################################
    
    for n, frame in enumerate(frame_list):    

        previous_mov_angle = (movement_angle[0], movement_angle[1], movement_angle[2])
                
        if (n > 0):
                        
            orientation = frame[1]
            movement_angle = mov_dir(orientation, frame[2])
            movement_angle = tuple(map(operator.add, movement_angle, previous_mov_angle))
                                                                                                
            if frame[3] == 'Revolute Joint':
        
                radius = 10
                height = 20

                Ut3D.drawCylinder(ax, x, y, z, radius, height, orientation, movement_angle)
                Ut3D.drawFrameAxis(ax, x, y, z, orientation, movement_angle)
        
            elif frame[3] == 'Prismatic Joint':

                L = 20

                Ut3D.drawCube(ax, x, y, z, L, movement_angle)
                Ut3D.drawFrameAxis(ax, x, y, z, orientation, movement_angle)
                
            else:
                print('Wrong Joint Type Defined.')
                
            link_length = frame[0]
            x, y, z = drawLink(ax, x, y, z, link_length, frame[3], orientation, movement_angle)
            
    return x, y, z

