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
# Start connection and Simulation with CoppeliaSim

CoppeliaSim = Coppelia()
CoppeliaSim.start_Simulation()

# Load Mobile Robot Pioneer 3DX
P = Pioneer3DX(CoppeliaSim.clientID)



P.send_ControlSignals([0.2,0])



# Main Routine
#CoppeliaSim.run_Simulation(10)

startTime=time.time()
while time.time()-startTime < 10:


    P.get_PositionData()
    time.sleep(0.1)


CoppeliaSim.stop_Simulation()


