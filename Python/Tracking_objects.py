# Make sure you have imported Coppelia.py and Pioneer3DX.py.
# Before run this code, open the CoppeliaSim simulator 
# and load the file scene.ttt
# This example starts the simulation by itself.
# The Circle_Trajectory.py shows how to control Pioneer 3DX in a circle,
# a mobile differencial drive robot.
# The controller applied is based on Lyapunov Theory.

from Coppelia import Coppelia
from Pioneer3DX import Pioneer3DX
from LaserSensor import LaserSensor
import matplotlib.pyplot as plt
import time
import numpy as np

# Load CoppeliaSim Class and Start Run Simulation
CoppeliaSim = Coppelia()
CoppeliaSim.start_Simulation()

# Load Mobile Robot Pioneer 3DX
Pioneer3DX = Pioneer3DX(CoppeliaSim.clientID)

# Load Laser Scanner
Laser = LaserSensor(CoppeliaSim.clientID)

# Config plot
shouldPlot = True
realRobotTraject_x, realRobotTraject_y = [], []
fig, ax = plt.subplots()
laserPointsPlot, = ax.plot(realRobotTraject_x, realRobotTraject_y, 'bo', ms=2)
#realRobotTrajectPlot, = ax.plot(realRobotTraject_x, realRobotTraject_y, 'k-')
degrees = np.array(range(0,181))
degrees = degrees - 90
PolarGraph, = ax.plot(realRobotTraject_x,realRobotTraject_y, 'b-')
Tracking_first, = ax.plot(realRobotTraject_x,realRobotTraject_y, 'ro')
Tracking_last, = ax.plot(realRobotTraject_x,realRobotTraject_y, 'ro')
ax.set_xlim(-6,6)
ax.set_ylim(-6,6)
fig.show()

## Main Routine
X_Desired = [0,0]
X_diff = [0,0]
# Start time routine
startTime=time.time()
while time.time()-startTime <30:
    t = time.time()-startTime

    # Set desired trajectory
    X_Desired = Pioneer3DX.get_curr_desired_point_CIRCLE(t)
    
    # Get Real Position From Robot
    # @remove avoid accessing directly class properties, the get method is for this purpose
    X_currRealPos, X_currRealOrientation = Pioneer3DX.get_PositionData()

    # Differential discrete
    # @todo generalized to use the [x_1,x_2,x_3], that is, the third coordinator
    X_diff = np.array([X_Desired - X_currRealPos[0:2]])
    
    # Get direct kinematic (for differential drive robot)
    Kinematic_matrix = Pioneer3DX.get_K_diff_drive_robot(X_currRealOrientation)

    Xtil = np.array([X_Desired - X_currRealPos[0:2]])
    # Get control signal from Lyapunov Control
    Ud = Pioneer3DX.lyapunov_controller_signal(Kinematic_matrix, X_diff, Xtil.transpose())
    
    
    # Get Laser Scanner data
    currLaserDataX, currLaserDataY = Laser.get_LaserData(X_currRealPos[0:2],X_currRealOrientation)

    #Tracking objects
    Derivative = []
    Polar_dist = np.hypot(currLaserDataX-X_currRealPos[0],currLaserDataY-X_currRealPos[1])
    Derivative = np.zeros([len(Polar_dist)])
    for i in range(0, len(Polar_dist)-1):
        Derivative[i] = Polar_dist[i+1] -Polar_dist[i] 

    #Derivative = np.gradient(Polar_dist)
    First_vert = []
    Last_vert = []
    for i in range(0,len(Derivative)):
        if Derivative[i]<-0.5:
            First_vert.append(i+1)
        elif Derivative[i]>0.5:
            Last_vert.append(i)

    #print(Derivative[First_vert])
    #print(Derivative[Last_vert])

    #print(First_vert)
    #print(Last_vert)
    #print(Derivative)

    # Send control signal to Pioneer
    Pioneer3DX.send_ControlSignals(Ud)

    # flag to activate plot
    if shouldPlot:
        #print('*** PLOT')
        # Save the X and Y robot coordinates and...
        #realRobotTraject_x.append(X_currRealPos[0])
        #realRobotTraject_y.append(X_currRealPos[1])
        # ... update plot
        laserPointsPlot.set_data(currLaserDataX, currLaserDataY)
        #realRobotTrajectPlot.set_data(realRobotTraject_x,realRobotTraject_y)
        
        #PolarGraph.set_data(degrees,Derivative)
        Tracking_first.set_data(currLaserDataX[First_vert],currLaserDataY[First_vert])
        Tracking_last.set_data(currLaserDataX[Last_vert],currLaserDataY[Last_vert])
        fig.canvas.draw()
        plt.pause(0.1)
    else:
        time.sleep(0.1)


# Stop the Simulation when you finish the routine
CoppeliaSim.stop_Simulation()


