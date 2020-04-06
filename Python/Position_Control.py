# Make sure you have imported Coppelia.py and Pioneer3DX.py.
# Before run this code, open the CoppeliaSim simulator 
# and load the file scene.ttt
# This example starts the simulation by itself.
# The MainRoutine.py shows how to control Pioneer 3DX,
# a mobile differencial drive robot.
# The controller applied is based on Lyapunov Theory.

from Coppelia import Coppelia
from Pioneer3DX import Pioneer3DX
import matplotlib.pyplot as plt
import time
import numpy as np

# Load CoppeliaSim Class and Start Run Simulation
CoppeliaSim = Coppelia()
CoppeliaSim.start_Simulation()

# Load Mobile Robot Pioneer 3DX
Pioneer3DX = Pioneer3DX(CoppeliaSim.clientID)

## Main Routine
# Start time routine
startTime=time.time()
while time.time()-startTime < 15:

    # Set desired Position
    X_Desired = [-2, 0]

    # Get Real Position From Robot
    X_currRealPos, X_currRealOrientation = Pioneer3DX.get_PositionData()

    # Differential discrete
    X_diff = np.array([[0,0]])
    
    # Get direct kinematic (for differential drive robot)
    Kinematic_matrix = Pioneer3DX.get_K_diff_drive_robot(X_currRealOrientation)

    # Get position error 
    Xtil = np.array([X_Desired - X_currRealPos[0:2]])

    # Get control signal from Lyapunov Control Ud = [linear,algular]
    Ud = Pioneer3DX.lyapunov_controller_signal(Kinematic_matrix, X_diff, Xtil.transpose())
  
    # Send control signal to Pioneer
    Pioneer3DX.send_ControlSignals(Ud)

    time.sleep(0.1)



# Stop the Simulation when you finish the routine
CoppeliaSim.stop_Simulation()


