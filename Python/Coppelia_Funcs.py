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
        
    return clientID


def stop_Simulation(clientID):
    #stop 
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
    print ('Program ended')

    return("Sucess!")



def load_Pioneer3DX(clientID):

    Pioneer3DX = [None] * 19
    error,Pioneer3DX[0] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking)
    error,Pioneer3DX[1] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
    error,Pioneer3DX[2] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)

    Pioneer3DX_sensors = "Pioneer_p3dx_ultrasonicSensor"
    sensor = 1
    while sensor < 17:
        error,Pioneer3DX[2+sensor] = sim.simxGetObjectHandle(clientID,Pioneer3DX_sensors+str(sensor),sim.simx_opmode_blocking)
        sensor+=1

    return Pioneer3DX


def send_ControlSignals(Ud,clientID, Pioneer3DX):
    # left_speed = (v - e w)/R 
    # right_speed = (v + e w)/R
    # v is the linear velocity given as parameter
    # w is the angular velocity given as parameter
    # e is half of the distance between the left and right wheels
    # R is the radius of the wheels
    R= 0.0925;  #raio da roda do Pioneer
    e = 0.17;   #distancia do centro até a roda

    left_speed  = (Ud[0] - e*Ud[1])/R
    right_speed = (Ud[0] + e*Ud[1])/R

    error = sim.simxSetJointTargetVelocity(clientID,Pioneer3DX[1],left_speed,sim.simx_opmode_streaming)
    error_2 = sim.simxSetJointTargetVelocity(clientID,Pioneer3DX[2],right_speed,sim.simx_opmode_streaming)
    #