import sim
import numpy as np

class LaserSensor:

    def __init__(self, clientID):
        # Connection ID to communicate with simulator
        self.clientID = clientID
        self.LaserDataX = np.zeros(181)
        self.LaserDataY = np.zeros(181)
        # Get Data from Laser Scanner on V-REP
        error, data = sim.simxGetStringSignal(self.clientID,'communication',sim.simx_opmode_streaming)
        
    def get_LaserData(self):
        ## Get Data from Laser Scanner on V-REP
        error, data = sim.simxGetStringSignal(self.clientID,'communication',sim.simx_opmode_buffer)

        # Unpack data to array
        array = sim.simxUnpackFloats(data)
        if len(array)>181:
            for i in range(0, 181):
                # Separate X and Y coordinates
                self.LaserDataX[i] = array[3*i+1]
                self.LaserDataY[i] = array[3*i+2]

            