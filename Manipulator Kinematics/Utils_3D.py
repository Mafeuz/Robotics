###################### 3d Plot Utils ######################

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
import numpy as np
import operator

####################################################################################################################################
def Rot(thetaRX, thetaRY, thetaRZ):

    thetaRX = np.radians(thetaRX)
    thetaRY = np.radians(thetaRY)
    thetaRZ = np.radians(thetaRZ)

    RX = np.array([[1, 0, 0],
          [0, np.cos(thetaRX), np.sin(thetaRX)],
          [0, -np.sin(thetaRX), np.cos(thetaRX)]])

    RY = np.array([[np.cos(thetaRY), 0, -np.sin(thetaRY)],
          [0, 1, 0],
          [np.sin(thetaRY), 0, np.cos(thetaRY)]])

    RZ = np.array([[np.cos(thetaRZ), np.sin(thetaRZ), 0],
          [-np.sin(thetaRZ), np.cos(thetaRZ), 0],
          [0, 0, 1]])

    # Transposition in order to keep the rotations related to the reference:
    RX = np.transpose(RX)
    RY = np.transpose(RY)
    RZ = np.transpose(RZ)

    R = np.matmul(RZ, RY)
    R = np.matmul(R, RX)

    return R

####################################################################################################################################
def vec_to_quiver(Origin, Vector):
    
    # This function will prepare a (X,Y,Z) array vector coordinates to quiver format (X,Y,Z,U,V,W)
    # considering the determined origin coordinates.
    
    vector_for_quiver = np.array([Origin[0], Origin[1], Origin[2], Vector[0], Vector[1], Vector[2]])
    
    return vector_for_quiver

####################################################################################################################################

def mov_vector_between(Origin_1, Origin_2, Vec_1, Vec_2):
    
    # This function is used to get the needed array to plot the movement of a vector using "quiver" with 3D matplotlib
    # In order to plot a moving vector between two moving vectors
    # Vec Format = np.array((X,Y,Z))
    
    # Origin_1 = The array with the XYZ coordinates of the first vector Origin
    # Origin_2 = The array with the XYZ coordinates of the second vector Origin
    # Vec_1 = Calculated Vector 1
    # Vec_1 = Calculated Vector 2
    
    # Vector Movimentation Aux:
    t1 = Origin_1[0] + Vec_1[0]
    t2 = Origin_1[1] + Vec_1[1]
    t3 = Origin_1[2] + Vec_1[2]
    
    d1 = (Origin_2[0] + Vec_2[0]) - (Origin_1[0] + Vec_1[0])
    d2 = (Origin_2[1] + Vec_2[1]) - (Origin_1[1] + Vec_1[1])
    d3 = (Origin_2[2] + Vec_2[2]) - (Origin_1[2] + Vec_1[2])
    
    vector_between_for_quiver = np.array([(t1),(t2),(t3),(d1),(d2),(d3)])

    return vector_between_for_quiver

####################################################################################################################################

def constructFaces(cX, cY, cZ, radius, max_theta, H, orientation, angle):
    
    # Draw Faces:
    x = np.array([])
    y = np.array([])
    
    for i in range(0, radius, 2):
        for theta in range(0, max_theta, 5):
            
            X = np.array([i*np.cos(np.radians(theta))])
            Y = np.array([i*np.sin(np.radians(theta))])
            
            x = np.concatenate((x, X))
            y = np.concatenate((y, Y))
            
    z = np.ones_like(x)*(H)
    
    RX, RY, RZ = orientation
        
    for i in range(x.shape[0]):
        x[i], y[i], z[i] = Rot(RX, RY, RZ).dot(np.array([x[i], y[i], z[i]]))
        
    if (angle != (0,0,0)):
        
        RX, RY, RZ = angle
        
        for i in range(x.shape[0]):
            x[i], y[i], z[i] = Rot(RX, RY, RZ).dot(np.array([x[i], y[i], z[i]]))
        
    x = x + cX
    y = y + cY
    z = z + cZ
    
    return x, y, z

####################################################################################################################################

def drawCylinder(ax, cX, cY, cZ, radius, height, orientation, angle):
    
    z = np.linspace(0, height, 16)
    theta = np.linspace(0, 2*np.pi, 16)
    theta_grid, z_grid=np.meshgrid(theta, z)
    
    x_grid = radius*np.cos(theta_grid)
    y_grid = radius*np.sin(theta_grid)
    z_grid = z_grid - height/2
    
    RX, RY, RZ = orientation
        
    # Rotate Cylinder with Angle(RX,RY,RZ):
    for i in range(z_grid.shape[0]):
        for j in range(z_grid.shape[0]):
            x_grid[i][j], y_grid[i][j], z_grid[i][j] = Rot(RX, RY, RZ).dot(np.array([x_grid[i][j], y_grid[i][j], z_grid[i][j]]))
            
    if (angle != (0,0,0)):
        
        RX, RY, RZ = angle
        
        # Rotate Cylinder with Angle(RX,RY,RZ):
        for i in range(z_grid.shape[0]):
            for j in range(z_grid.shape[0]):
                x_grid[i][j], y_grid[i][j], z_grid[i][j] = Rot(RX, RY, RZ).dot(np.array([x_grid[i][j], y_grid[i][j], z_grid[i][j]]))
    
    x_grid = x_grid + cX
    y_grid = y_grid + cY
    z_grid = z_grid + cZ

    ##################################################################################################################        
    
    xUp, yUp, zUp = constructFaces(cX, cY, cZ, radius, 360, height/2, orientation, angle)
    xDown, yDown, zDown = constructFaces(cX, cY, cZ, radius, 360, -height/2, orientation, angle)
    
    ax.plot(xUp, yUp, zUp, alpha = 0.5, color='blue')
    ax.plot(xDown, yDown, zDown, alpha = 0.5, color = 'blue')
    
    movX, movY, movZ = constructFaces(cX, cY, cZ, radius, 45, height/2, orientation, angle)
    ax.plot(movX, movY, movZ, alpha = 1, color = 'white')
    
    ##################################################################################################################
    
    ax.plot_surface(x_grid, y_grid, z_grid, alpha=1)
    
####################################################################################################################################
def drawCube(ax, cX, cY, cZ, L, angle, color='black', rec=1):
    
    # Draw Faces:
    x = np.array([])
    y = np.array([])
    z = np.array([])
    
    for i in range(0, L, 2):
        for j in range(0, L, 2):
            for k in range(0, L, 2):
            
                X = np.array([i])
                Y = np.array([j])
                Z = np.array([k])

                x = np.concatenate((x, X))
                y = np.concatenate((y, Y))
                z = np.concatenate((z, Z))
                
    x = x -L/2
    y = y -L/2
    z = z -L/2
        
    if (angle !=(0,0,0)):
        
        RX, RY, RZ = angle
        
        for i in range(x.shape[0]):
            x[i], y[i], z[i] = Rot(RX, RY, RZ).dot(np.array([x[i], y[i], z[i]]))
            
    x = x + cX
    y = y + cY
    z = z + cZ
    
    ax.plot(x, y, z, alpha = 0.9, color = color)
        
    if (rec==1):
        drawCube(ax, cX, cY, cZ, L-4, angle, color='blue', rec=0)
        
####################################################################################################################################
def drawFrameAxis(ax, cX, cY, cZ, orientation, angle):
    
    framePos = np.array([cX, cY, cZ])
    
    RX, RY, RZ = orientation
    
    baseX = np.array([30,0,0])
    baseY = np.array([0,30,0])
    baseZ = np.array([0,0,30])
    
    baseX = Rot(RX, RY, RZ).dot(baseX)
    baseY = Rot(RX, RY, RZ).dot(baseY)
    baseZ = Rot(RX, RY, RZ).dot(baseZ)
    
    RX, RY, RZ = angle
    
    frame_axisX = Rot(RX, RY, RZ).dot(baseX)
    frame_axisY = Rot(RX, RY, RZ).dot(baseY)
    frame_axisZ = Rot(RX, RY, RZ).dot(baseZ)
    
    XaxisX, YaxisX, ZaxisX, UaxisX, VaxisX, WaxisX = vec_to_quiver(framePos, frame_axisX)
    XaxisY, YaxisY, ZaxisY, UaxisY, VaxisY, WaxisY = vec_to_quiver(framePos, frame_axisY)
    XaxisZ, YaxisZ, ZaxisZ, UaxisZ, VaxisZ, WaxisZ = vec_to_quiver(framePos, frame_axisZ)

    ax.quiver(XaxisX, YaxisX, ZaxisX, UaxisX, VaxisX, WaxisX, arrow_length_ratio = 0.2, color='r')
    ax.quiver(XaxisY, YaxisY, ZaxisY, UaxisY, VaxisY, WaxisY, arrow_length_ratio = 0.2, color='g')
    ax.quiver(XaxisZ, YaxisZ, ZaxisZ, UaxisZ, VaxisZ, WaxisZ, arrow_length_ratio = 0.2, color='b')
    
    

