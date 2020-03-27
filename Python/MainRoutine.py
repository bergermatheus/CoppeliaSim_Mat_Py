# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

from Coppelia import Coppelia
from Pioneer3DX import Pioneer3DX
import time
import numpy as np
# Start connection and Simulation with CoppeliaSim

CoppeliaSim = Coppelia()
CoppeliaSim.start_Simulation()

# Load Mobile Robot Pioneer 3DX
P = Pioneer3DX(CoppeliaSim.clientID)



P.send_ControlSignals([0.2,0])



## Main Routine
# Start time routine
startTime=time.time()
while time.time()-startTime < 30:

    # Set Position Desired
    X_Desired = [-2, 3]

    # Get Real Position From Robot
    P.get_PositionData()
    
    # Direct kinematic
    K = np.array([[np.cos(P.orientation),-0.15*np.sin(P.orientation)],[np.sin(P.orientation),0.15*np.cos(P.orientation)]])
    
    # Position Error
    Xtil = np.array([X_Desired - P.position_coordX[0:2]])
    Xtil = Xtil.transpose()

    a = np.linalg.inv(K)
    b = 0.7*np.tanh(0.5*Xtil)
    # Controller
    Ud = np.dot(a,b)

    # Sendo control signal to Pioneer
    P.send_ControlSignals(Ud)

    time.sleep(0.1)


CoppeliaSim.stop_Simulation()


