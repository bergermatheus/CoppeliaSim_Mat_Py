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

from Coppelia_Funcs import *
from CoppeliaTest import CoppeliaTest
from Pioneer3DXTest import Pioneer3DX
# Start connection and Simulation with CoppeliaSim

CoppeliaSim = CoppeliaTest()
CoppeliaSim.start_Simulation()

# Load Mobile Robot Pioneer 3DX
P= Pioneer3DX(CoppeliaSim.clientID)



send_ControlSignals([0.2,0],CoppeliaSim.clientID, P.pioneer3DX_array)



# Main Routine
startTime=time.time()
while time.time()-startTime < 10:
    
    time.sleep(0.1)


CoppeliaSim.stop_Simulation()


