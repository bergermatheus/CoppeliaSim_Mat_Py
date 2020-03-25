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

clientID = start_Simulation()


# Now retrieve streaming data (i.e. in a non-blocking fashion):
startTime=time.time()
sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming
while time.time()-startTime < 5:
    returnCode,data=sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_buffer) # Try to retrieve the streamed data
    if returnCode==sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        print ('Mouse position x: ',data) # Mouse position x is actualized when the cursor is over CoppeliaSim's window
    time.sleep(0.005)


stop_Simulation(clientID)


