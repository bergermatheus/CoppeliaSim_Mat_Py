# Make sure you have imported Coppelia.py and Pioneer3DX.py.
# Before run this code, open the CoppeliaSim simulator 
# and load the file scene.ttt
# This example starts the simulation by itself.
# The Avoidance_algorithm.py shows how to implement a
# obstacle avoidance strategy, using Potencial Fields.
# The controller applied is based on Lyapunov Theory.

## Import the libraries you need
from Coppelia import Coppelia
from Pioneer3DX import Pioneer3DX
from LaserSensor import LaserSensor
import matplotlib.pyplot as plt
from Navigation import *
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
realRobotTrajectPlot, = ax.plot(realRobotTraject_x, realRobotTraject_y, 'k-')
RobotPosition, = ax.plot(realRobotTraject_x, realRobotTraject_y, 'ro')
Goal, = ax.plot(realRobotTraject_x, realRobotTraject_y, 'go')
ax.set_xlim(-6,6)
ax.set_ylim(-6,6)
fig.show()

## Main Routine
X_Desired = [0,0]
X_diff = [0,0]

# Start time routine
startTime=time.time()
while time.time()-startTime < 40:
    t = time.time()-startTime

    # Set desired trajectory
    #X_Desired = get_curr_desired_point_CIRCLE(t)
    X_Desired = [3.15, 0]

    # Get Real Position From Robot
    X_currRealPos, X_currRealOrientation = Pioneer3DX.get_PositionData()

    # Differential discrete
    X_diff = np.array([X_Desired - X_currRealPos[0:2]])
    
    # Get direct kinematic (for differential drive robot)
    Kinematic_matrix = Pioneer3DX.get_K_diff_drive_robot(X_currRealOrientation)

    # Get position error 
    Xtil = np.array([X_Desired - X_currRealPos[0:2]])

    # Get control signal from Lyapunov Control Ud = [linear,algular]
    Ud = Pioneer3DX.lyapunov_controller_signal(Kinematic_matrix, X_diff, Xtil.transpose())

    
    # Get Laser Scanner data
    currLaserDataX, currLaserDataY = Laser.get_LaserData(X_currRealPos[0:2],X_currRealOrientation)
    
    # Apply avoidance Algorithm (Potencial Field)
    flag_avoid, Force_result = avoidance_Potencial_Field(currLaserDataX,currLaserDataY,X_currRealPos,X_Desired)
    
    # If the distance from the robot to the obstacle is closer than 0.8m
    if flag_avoid:
        X_diff = np.array([[0,0]])
        Ud = Pioneer3DX.lyapunov_controller_signal(Kinematic_matrix, X_diff, Force_result)
        Pioneer3DX.send_ControlSignals(Ud)
        # Disable flag to recalculate the distance
        flag_avoid = False
    else:
        # Send control signal to Pioneer
        Pioneer3DX.send_ControlSignals(Ud)

    # flag to activate plot
    if shouldPlot:
        # Save the X and Y robot coordinates and...
        realRobotTraject_x.append(X_currRealPos[0])
        realRobotTraject_y.append(X_currRealPos[1])
        # ... update plot
        # Laser plot
        laserPointsPlot.set_data(currLaserDataX, currLaserDataY)
        # Robot track
        realRobotTrajectPlot.set_data(realRobotTraject_x,realRobotTraject_y)
        # Goal point
        Goal.set_data(X_Desired[0],X_Desired[1])
        RobotPosition.set_data(X_currRealPos[0],X_currRealPos[1])
        fig.canvas.draw()
        plt.pause(0.1)
    else:
        time.sleep(0.1)


# Stop the Simulation when you finish the routine
CoppeliaSim.stop_Simulation()


