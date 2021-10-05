####################################################################################################################################
####################################################################################################################################
################################# Manipulator Kinematics Simulator Functions by Matheus D. Pereira #################################

import numpy as np
import cv2
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

####################################################################################################################################
def constructFaces(cX, cY, cZ, radius, max_theta, H, orientation, self_angle, previous_angle):
    
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
        
    self_angle = mov_dir(orientation, self_angle)
                
    RX, RY, RZ = self_angle
            
    for i in range(x.shape[0]):
        x[i], y[i], z[i] = Rot(RX, RY, RZ).dot(np.array([x[i], y[i], z[i]]))
        
    RX, RY, RZ = previous_angle
        
    for i in range(x.shape[0]):
        x[i], y[i], z[i] = Rot(RX, RY, RZ).dot(np.array([x[i], y[i], z[i]]))
        
    x = x + cX
    y = y + cY
    z = z + cZ
    
    return x, y, z

####################################################################################################################################
def drawCylinder(ax, cX, cY, cZ, radius, height, orientation, self_angle, previous_angle):
    
    z = np.linspace(0, height, 16)
    theta = np.linspace(0, 2*np.pi, 16)
    theta_grid, z_grid=np.meshgrid(theta, z)
    
    x_grid = radius*np.cos(theta_grid)
    y_grid = radius*np.sin(theta_grid)
    z_grid = z_grid - height/2
    
    RX, RY, RZ = orientation
        
    for i in range(z_grid.shape[0]):
        for j in range(z_grid.shape[0]):
            x_grid[i][j], y_grid[i][j], z_grid[i][j] = Rot(RX, RY, RZ).dot(np.array([x_grid[i][j], y_grid[i][j], z_grid[i][j]]))
                    
    self_angle_adj = mov_dir(orientation, self_angle)        
    
    RX, RY, RZ = self_angle_adj
        
    for i in range(z_grid.shape[0]):
        for j in range(z_grid.shape[0]):
            x_grid[i][j], y_grid[i][j], z_grid[i][j] = Rot(RX, RY, RZ).dot(np.array([x_grid[i][j], y_grid[i][j], z_grid[i][j]]))
            
    RX, RY, RZ = previous_angle
        
    for i in range(z_grid.shape[0]):
        for j in range(z_grid.shape[0]):
            x_grid[i][j], y_grid[i][j], z_grid[i][j] = Rot(RX, RY, RZ).dot(np.array([x_grid[i][j], y_grid[i][j], z_grid[i][j]]))
    
    x_grid = x_grid + cX
    y_grid = y_grid + cY
    z_grid = z_grid + cZ

    ##################################################################################################################        
    
    xUp, yUp, zUp = constructFaces(cX, cY, cZ, radius, 360, height/2, orientation, self_angle, previous_angle)
    xDown, yDown, zDown = constructFaces(cX, cY, cZ, radius, 360, -height/2, orientation, self_angle, previous_angle)
    
    ax.plot(xUp, yUp, zUp, alpha = 0.5, color='blue')
    ax.plot(xDown, yDown, zDown, alpha = 0.5, color = 'blue')
    
    movX, movY, movZ = constructFaces(cX, cY, cZ, radius, 45, height/2, orientation, self_angle, previous_angle)
    ax.plot(movX, movY, movZ, alpha = 1, color = 'white')
    
    ##################################################################################################################
    
    ax.plot_surface(x_grid, y_grid, z_grid, alpha=1)
    
####################################################################################################################################
def drawCube(ax, cX, cY, cZ, L, orientation, self_angle, previous_angle, color='black', rec=1):
    
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
        
    self_angle_adj = mov_dir(orientation, self_angle)
        
    RX, RY, RZ = self_angle_adj
        
    for i in range(x.shape[0]):
        x[i], y[i], z[i] = Rot(RX, RY, RZ).dot(np.array([x[i], y[i], z[i]]))
            
    RX, RY, RZ = previous_angle
        
    for i in range(x.shape[0]):
        x[i], y[i], z[i] = Rot(RX, RY, RZ).dot(np.array([x[i], y[i], z[i]]))
                                
    x = x + cX
    y = y + cY
    z = z + cZ
    
    ax.plot(x, y, z, alpha = 0.9, color = color)
        
    if (rec==1):
        drawCube(ax, cX, cY, cZ, L-4, orientation, self_angle, previous_angle, color='blue', rec=0)
        
####################################################################################################################################
def drawFrameAxis(ax, cX, cY, cZ, orientation, self_angle, previous_angle):
    
    framePos = np.array([cX, cY, cZ])
    
    RX, RY, RZ = orientation
    
    baseX = np.array([30,0,0])
    baseY = np.array([0,30,0])
    baseZ = np.array([0,0,30])
    
    baseX = Rot(RX, RY, RZ).dot(baseX)
    baseY = Rot(RX, RY, RZ).dot(baseY)
    baseZ = Rot(RX, RY, RZ).dot(baseZ)
    
    self_angle_adj = mov_dir(orientation, self_angle)
    
    RX, RY, RZ = self_angle_adj
    
    baseX = Rot(RX, RY, RZ).dot(baseX)
    baseY = Rot(RX, RY, RZ).dot(baseY)
    baseZ = Rot(RX, RY, RZ).dot(baseZ)
    
    RX, RY, RZ = previous_angle
    
    baseX = Rot(RX, RY, RZ).dot(baseX)
    baseY = Rot(RX, RY, RZ).dot(baseY)
    baseZ = Rot(RX, RY, RZ).dot(baseZ)
    
    XaxisX, YaxisX, ZaxisX, UaxisX, VaxisX, WaxisX = vec_to_quiver(framePos, baseX)
    XaxisY, YaxisY, ZaxisY, UaxisY, VaxisY, WaxisY = vec_to_quiver(framePos, baseY)
    XaxisZ, YaxisZ, ZaxisZ, UaxisZ, VaxisZ, WaxisZ = vec_to_quiver(framePos, baseZ)

    ax.quiver(XaxisX, YaxisX, ZaxisX, UaxisX, VaxisX, WaxisX, arrow_length_ratio = 0.2, color='r')
    ax.quiver(XaxisY, YaxisY, ZaxisY, UaxisY, VaxisY, WaxisY, arrow_length_ratio = 0.2, color='g')
    ax.quiver(XaxisZ, YaxisZ, ZaxisZ, UaxisZ, VaxisZ, WaxisZ, arrow_length_ratio = 0.2, color='b')
    
####################################################################################################################
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

####################################################################################################################

def drawLink(ax, x1, y1, z1, link_length, joint_type, orientation, self_angle, previous_angle):
    
    LinkArray = linkDirection(orientation, link_length)
    
    self_angle_adj = mov_dir(orientation, self_angle)
            
    RX, RY, RZ = self_angle_adj
    x2, y2, z2 = Rot(RX, RY, RZ).dot(LinkArray)
    
    RX, RY, RZ = previous_angle
    x2, y2, z2 = Rot(RX, RY, RZ).dot(np.array([x2, y2, z2])) + np.array([x1, y1, z1])
    
    origin = np.array([x1, y1, z1])
    vector = np.array([x2 - x1, y2 - y1, z2 - z1])
    
    X, Y, Z, U, V, W = vec_to_quiver(origin, vector)
    ax.quiver(X, Y, Z, U, V, W, arrow_length_ratio = 0.05, color='k')
    
    if (joint_type == 'Prismatic Joint'):
        ax.quiver(X, Y, Z, U, V, W, arrow_length_ratio = 0.1, color='y')
        ax.quiver(X+1, Y+1, Z+1, U, V, W, arrow_length_ratio = 0.1, color='y')
    
    return x2, y2, z2

####################################################################################################################

def kinematics_diagram(ax, frame_list):
    
    ############################################################################################################
    # Frame One:
    
    frameOne = frame_list[0]
    
    orientation = frameOne[3]
    self_angle = frameOne[4]
    previous_angle = (0,0,0)
    
    x, y, z = frameOne[0], frameOne[1], frameOne[2]
    
    ax.text(x, y, z, 'Frame 1', (0,0,0))

    if frameOne[5] == 'Revolute Joint':
        
        radius = 10
        height = 20
        
        drawCylinder(ax, x, y, z, radius, height, orientation, self_angle, previous_angle)
        drawFrameAxis(ax, x, y, z, orientation, self_angle, previous_angle)
        
    elif frameOne[5] == 'Prismatic Joint':
        
        L = 20
        
        drawCube(ax, x, y, z, L, orientation, self_angle, previous_angle)
        drawFrameAxis(ax, x, y, z, orientation, self_angle, previous_angle)
        
    else:
        print('Wrong Joint Type Defined.')
        
    link_length = frameOne[6]
    x, y, z = drawLink(ax, x, y, z, link_length, frameOne[5], orientation, self_angle, previous_angle)
    
    if (len(frame_list) > 1):
        ax.text(x, y, z, 'Frame 2', (0,0,0))
            
    ############################################################################################################
        
    for n, frame in enumerate(frame_list):
        
        if (n > 0):
            
            previous_angle_add = mov_dir(orientation, self_angle)
            previous_angle = tuple(map(operator.add, previous_angle_add, previous_angle))
                                            
            orientation = frame[1]
            self_angle = frame[2]
                                                                                                                                        
            if frame[3] == 'Revolute Joint':
        
                radius = 10
                height = 20

                drawCylinder(ax, x, y, z, radius, height, orientation, self_angle, previous_angle)
                drawFrameAxis(ax, x, y, z, orientation, self_angle, previous_angle)
        
            elif frame[3] == 'Prismatic Joint':

                L = 20

                drawCube(ax, x, y, z, L, orientation, self_angle, previous_angle)
                drawFrameAxis(ax, x, y, z, orientation, self_angle, previous_angle)
                
            else:
                print('Wrong Joint Type Defined.')
                
            link_length = frame[0]
            x, y, z = drawLink(ax, x, y, z, link_length, frame[3], orientation, self_angle, previous_angle)
            
        if (n + 1 < len(frame_list)):
            ax.text(x, y, z, 'Frame {}'.format(n+2), (0,0,0))
                
        if (n + 1 == len(frame_list)):
            ax.text(x, y, z, 'End Effector', (0,0,0))
            
    return x, y, z

####################################################################################################################
def config_axis(ax):
    
    Xmin, Xmax = 0, 400
    Ymin, Ymax = 0, 400
    Zmin, Zmax = 0, 200

    ax.set_xlabel('\n X ', fontsize=15)
    ax.set_ylabel('\n Y ', fontsize=15)
    ax.set_zlabel('\n Z ', fontsize=15)

    ax.set_xticks([0, 50, 100, 150, 200, 250, 300, 350, 400])
    ax.set_yticks([0, 50, 100, 150, 200, 250, 300, 350, 400])
    ax.set_zticks([0, 50, 100, 150, 200])
    ax.dist = 12

    ax.set_xlim([Xmin, Xmax])
    ax.set_ylim([Ymin, Ymax])
    ax.set_zlim([Zmin, Zmax])

    ax.set_title('Manipulator Kinematics Diagram', fontsize=15)

    ax.quiver(Xmin, 0, 0, Xmax, 0, 0, arrow_length_ratio = 0.05, color='k')
    ax.quiver(0, Ymin, 0, 0, Ymax, 0, arrow_length_ratio = 0.05, color='k')
    ax.quiver(0, 0, Zmin, 0, 0, Zmax, arrow_length_ratio = 0.05, color='k')
    
####################################################################################################################