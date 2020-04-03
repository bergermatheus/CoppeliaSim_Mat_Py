# Make sure you have imported Coppelia.py and Pioneer3DX.py.
# Before run this code, open the CoppeliaSim simulator 
# and load the file scene.ttt
# This example starts the simulation by itself.
# The MainRoutine.py shows how to control Pioneer 3DX,
# a mobile differencial drive robot.
# The controller applied is based on Lyapunov Theory.

from Coppelia import Coppelia
from Pioneer3DX import Pioneer3DX
from CameraSensor import CameraSensor
import matplotlib.pyplot as plt
import time
import numpy as np

# Load CoppeliaSim Class and Start Run Simulation
CoppeliaSim = Coppelia()
CoppeliaSim.start_Simulation()

# Load Mobile Robot Pioneer 3DX
Pioneer3DX = Pioneer3DX(CoppeliaSim.clientID)

# Setup Sensor Figure
# Load Camera Sensor
Camera = CameraSensor(CoppeliaSim.clientID)
time.sleep(2)

Resol_XY, image = Camera.get_SnapShot()
fig, ax = plt.subplots()
Image_object = ax.imshow(image)
fig.show()

#lt.imshow(image)
#plt.show()

shouldPlot = True

## Main Routine
X_Desired = [0,0]
X_diff = [0,0]
# Start time routine
startTime=time.time()
while time.time()-startTime < 90:
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

    # Send control signal to Pioneer
    Pioneer3DX.send_ControlSignals(Ud)

    # flag to activate plot
    if shouldPlot:
        Resol_XY, image = Camera.get_SnapShot()
        Image_object.set_data(image)
        fig.canvas.draw()
        plt.pause(0.1)
    else:
        time.sleep(0.1)


# Stop the Simulation when you finish the routine
CoppeliaSim.stop_Simulation()


