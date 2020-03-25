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

# left_speed = (v - e w)/R 
# right_speed = (v + e w)/R
# v is the linear velocity given as parameter
# w is the angular velocity given as parameter
# e is half of the distance between the left and right wheels
# R is the radius of the wheels
R= 0.0925;  #raio da roda do Pioneer
e = 0.17;   #distancia do centro até a roda
Ud = [0, 0.5]

left_speed  = (Ud[0] - e*Ud[1])/R
right_speed = (Ud[0] + e*Ud[1])/R

error = sim.simxSetJointTargetVelocity(clientID,Pioneer3DX[1],left_speed,sim.simx_opmode_streaming)
error_2 = sim.simxSetJointTargetVelocity(clientID,Pioneer3DX[2],right_speed,sim.simx_opmode_streaming)
#

# Main Routine
startTime=time.time()
while time.time()-startTime < 10:
    
    time.sleep(0.1)


stop_Simulation(clientID)


