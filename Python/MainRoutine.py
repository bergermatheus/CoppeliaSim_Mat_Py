# Make sure you have imported Coppelia.py and Pioneer3DX.py.
# Before run this code, open the CoppeliaSim simulator 
# and load the file scene.ttt
# This example starts the simulation by itself.
# The MainRoutine.py shows how to control Pioneer 3DX,
# a mobile differencial drive robot.
# The controller applied is based on Lyapunov Theory.

from Coppelia import Coppelia
from Pioneer3DX import Pioneer3DX
import time
import numpy as np

# Load CoppeliaSim Class and Start Run Simulation
CoppeliaSim = Coppelia()
CoppeliaSim.start_Simulation()

# Load Mobile Robot Pioneer 3DX
P = Pioneer3DX(CoppeliaSim.clientID)

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

    # Send control signal to Pioneer
    P.send_ControlSignals(Ud)

    time.sleep(0.1)

# Stop the Simulation when you finish the routine
CoppeliaSim.stop_Simulation()


