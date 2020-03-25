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
# Start connection and Simulation with CoppeliaSim
clientID = start_Simulation()

# Load Mobile Robot Pioneer 3DX
Pioneer3DX = load_Pioneer3DX(clientID)



send_ControlSignals([0.2,0],clientID, Pioneer3DX)



# Main Routine
startTime=time.time()
while time.time()-startTime < 10:
    
    time.sleep(0.1)


stop_Simulation(clientID)


