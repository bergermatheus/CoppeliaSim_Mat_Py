import sim
import time

class Coppelia:

    def __init__(self):
        #Library Python to connect with CoppeliaSim
        self.clientID = -1

    ## Start Connection with CoppeliaSim and Start Run Simulation
    def start_Simulation(self):

        print ('Program started')
        # Just in case, close all opened connections
        sim.simxFinish(-1) 
        # Connect to CoppeliaSim, as continuous remote API Server
        self.clientID = sim.simxStart('127.0.0.1',19997,False,True,5000,5)
        
        if self.clientID!=-1:
            print ('Connected to remote API server')
            #Start simulation
            sim.simxStartSimulation(self.clientID, sim.simx_opmode_oneshot)
            # Now send some data to CoppeliaSim in a non-blocking fashion:
            sim.simxAddStatusbarMessage(self.clientID,'Connection Succeed!',sim.simx_opmode_oneshot)
            # Before closing the connection to CoppeliaSim, make sure that the last command sent
            # out had time to arrive. You can guarantee this with (for example):
            sim.simxGetPingTime(self.clientID)

            # Now try to retrieve data in a blocking fashion (i.e. a service call):
            res,objs=sim.simxGetObjects(self.clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
            if res==sim.simx_return_ok:
                print ('Number of objects in the scene: ',len(objs))
            else:
                print ('Remote API function call returned with error code: ',res)

            time.sleep(2)
        else:
            print ('Failed connecting to remote API server')
            
        return self.clientID

    def stop_Simulation(self):
        #Stop Run Simulation 
        sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot)

        # Before closing the connection to CoppeliaSim, make sure that the last 
        # command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(self.clientID)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(self.clientID)
        print ('Program ended')

        return None 


    def run_Simulation(self, time_interval):
        startTime=time.time()
        while time.time()-startTime < time_interval:

            time.sleep(0.1)

