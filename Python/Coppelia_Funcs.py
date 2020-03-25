try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

def start_Simulation():

    print ('Program started')
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19997,False,True,5000,5) # Connect to CoppeliaSim, as continuous remote API Server
    if clientID!=-1:
        print ('Connected to remote API server')
        #start simulation
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
        # Now send some data to CoppeliaSim in a non-blocking fashion:
        sim.simxAddStatusbarMessage(clientID,'Connection Succeed!',sim.simx_opmode_oneshot)
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)
        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
        if res==sim.simx_return_ok:
            print ('Number of objects in the scene: ',len(objs))
        else:
            print ('Remote API function call returned with error code: ',res)

        time.sleep(2)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')
    
    return clientID


def stop_Simulation(clientID):
    #stop 
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)

    return("Sucess!")