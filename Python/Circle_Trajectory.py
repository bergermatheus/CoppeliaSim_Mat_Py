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

# Define Direct Kinematic (for differential drive robot)
def get_K_diff_drive_robot(X_currRealOrientation):
    # K = [[cos(theta)  -0.15*sin(theta)
    #       sin(theta)   0.15*cos(theta)]]
    K = np.array([[np.cos(X_currRealOrientation),-0.15*np.sin(X_currRealOrientation)],[np.sin(X_currRealOrientation),0.15*np.cos(X_currRealOrientation)]])
    return K

# Load Laser Scanner
Laser = LaserSensor(CoppeliaSim.clientID)

# Define controller signal
def lyapunov_controller_signal(kinematic, X_diff, Xtil):
    # Lyapunov Controller: Ud = K^-1*(0.4*X_diff + 0.7*tanh(0.5Xtil))
    # Inverse kinematic K^-1
    a = np.linalg.inv(kinematic)
    b = 0.3*X_diff.transpose() + 1.2*np.tanh(0.8*Xtil)
    Ud = np.dot(a,b)
    return Ud

# Define trajectory
def get_curr_desired_point_CIRCLE(tStep):
    # Parameters of the circle
    r = 1.5
    T = 30.0
    w = 1/T
    return [r * np.cos(2*np.pi*w * tStep), r * np.sin(2*np.pi*w * tStep)]

# Config plot
shouldPlot = True
realRobotTraject_x, realRobotTraject_y = [], []
fig, ax = plt.subplots()
laserPointsPlot, = ax.plot(realRobotTraject_x, realRobotTraject_y, 'bo', ms=2)
realRobotTrajectPlot, = ax.plot(realRobotTraject_x, realRobotTraject_y, 'k-')
ax.set_xlim(-6,6)
ax.set_ylim(-6,6)
fig.show()

## Main Routine
X_Desired = [0,0]
X_diff = [0,0]
# Start time routine
startTime=time.time()
while time.time()-startTime < 30:
    t = time.time()-startTime

    # Set desired trajectory
    X_Desired = get_curr_desired_point_CIRCLE(t)
    
    # Get Real Position From Robot
    # @remove avoid accessing directly class properties, the get method is for this purpose
    X_currRealPos, X_currRealOrientation = Pioneer3DX.get_PositionData()

    # Differential discrete
    # @todo generalized to use the [x_1,x_2,x_3], that is, the third coordinator
    X_diff = np.array([X_Desired - X_currRealPos[0:2]])
    
    # Get direct kinematic (for differential drive robot)
    Kinematic_matrix = get_K_diff_drive_robot(X_currRealOrientation)

    Xtil = np.array([X_Desired - X_currRealPos[0:2]])
    # Get control signal from Lyapunov Control
    Ud = lyapunov_controller_signal(Kinematic_matrix, X_diff, Xtil.transpose())
    # Send control signal to Pioneer
    Pioneer3DX.send_ControlSignals(Ud)
    
    # Get Laser Scanner data
    currLaserDataX, currLaserDataY = Laser.get_LaserData(X_currRealPos[0:2],X_currRealOrientation)

    # flag to activate plot
    if shouldPlot:
        #print('*** PLOT')
        # Save the X and Y robot coordinates and...
        realRobotTraject_x.append(X_currRealPos[0])
        realRobotTraject_y.append(X_currRealPos[1])
        # ... update plot
        laserPointsPlot.set_data(currLaserDataX, currLaserDataY)
        realRobotTrajectPlot.set_data(realRobotTraject_x,realRobotTraject_y)

        fig.canvas.draw()
        plt.pause(0.1)
    else:
        time.sleep(0.1)


# Stop the Simulation when you finish the routine
CoppeliaSim.stop_Simulation()


