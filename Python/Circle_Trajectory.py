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

## Setting plot 

x, y = [], []

fig, ax = plt.subplots()
points, = ax.plot(x, y, 'bo', ms=2)
#points1, = ax.plot(x, y, 'k-')
ax.set_xlim(-6,6)
ax.set_ylim(-6,6)
fig.show()

# Load CoppeliaSim Class and Start Run Simulation
CoppeliaSim = Coppelia()
CoppeliaSim.start_Simulation()

# Load Mobile Robot Pioneer 3DX
P = Pioneer3DX(CoppeliaSim.clientID)

# Load Laser Scanner
L = LaserSensor(CoppeliaSim.clientID)



## Main Routine
# Parameters of the circle
r = 1.5
T = 30
w = 1/T
X_Desired = [0,0]
X_diff = [0,0]
# Start time routine
startTime=time.time()
while time.time()-startTime < 30:
    t= time.time()-startTime

    # Set Circle Trajectory Desired
    X_Desired[0] = r * np.cos(2*np.pi*w * t)
    X_Desired[1] = r * np.sin(2*np.pi*w * t)
    

    # Get Real Position From Robot
    P.get_PositionData()

    # Differencial discrete
    X_diff = np.array([X_Desired - P.position_coordX[0:2]])
    # X_diff_transp = np.array([[X_diff[0]],[X_diff[1]]])
    

    # Direct kinematic for differencial drive robot
    # K = [[cos(theta)  -0.15*sin(theta)
    #       sin(theta)   0.15*cos(theta)]]
    K = np.array([[np.cos(P.orientation),-0.15*np.sin(P.orientation)],[np.sin(P.orientation),0.15*np.cos(P.orientation)]])
    
    # Position Error
    # Xtil = [Xdesired Ydesired] - [Xrobot Yrobot]
    Xtil = np.array([X_Desired - P.position_coordX[0:2]])
    Xtil = Xtil.transpose()

    # Inverse kinematic K^-1
    a = np.linalg.inv(K)
    # Lyapunov Controller: Ud = K^-1*(0.4*X_diff + 0.7*tanh(0.5Xtil))
    b = 0.3*X_diff.transpose() + 1.2*np.tanh(0.8*Xtil)
    Ud = np.dot(a,b)
    
    # Laser Scanner
    L.get_LaserData(P.position_coordX[0:2],P.orientation)

    # Save the X and Y coordenates and update plot
    x.append(P.position_coordX[0])
    y.append(P.position_coordX[1])
    
    #Update Dataset from Laser and Robot Position
    points.set_data(L.LaserDataX, L.LaserDataY)
    #points1.set_data(x,y)
    
    # Refresh Plot Image
    fig.canvas.draw()
    
    # Send control signal to Pioneer
    P.send_ControlSignals(Ud)

    #time.sleep(0.1)
    plt.pause(0.1)



# Stop the Simulation when you finish the routine
CoppeliaSim.stop_Simulation()


