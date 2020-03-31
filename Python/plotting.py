from Coppelia import Coppelia
from Pioneer3DX import Pioneer3DX
from LaserSensor import LaserSensor
import matplotlib.pyplot as plt
import time
import numpy as np


## Setting plot 

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlim(-5,5)
ax.set_ylim(-5,5)
fig.show()
x, y = [], []

# Load CoppeliaSim Class and Start Run Simulation
CoppeliaSim = Coppelia()
CoppeliaSim.start_Simulation()

# Load Mobile Robot Pioneer 3DX
P = Pioneer3DX(CoppeliaSim.clientID)
# Load Laser Scanner
L = LaserSensor(CoppeliaSim.clientID)
## Main Routine
# Start time routine
startTime=time.time()
while time.time()-startTime < 30:
    # Set Position Desired
    X_Desired = [-2, 3]

    # Get Real Position From Robot
    P.get_PositionData()
    
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
    # Lyapunov Controller: Ud = K^-1*(0.7*tanh(0.5Xtil))
    b = 0.7*np.tanh(0.5*Xtil)
    Ud = np.dot(a,b)

    # Laser Scanner
    L.get_LaserData()
    
    x.append(P.position_coordX[0])
    y.append(P.position_coordX[1])
    
    ax.plot(x, y, 'k-')
    
    fig.canvas.draw()
    

    # Send control signal to Pioneer
    P.send_ControlSignals(Ud)
    
    time.sleep(0.1)



# Stop the Simulation when you finish the routine
CoppeliaSim.stop_Simulation()
