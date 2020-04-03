import sim
import numpy as np

class CameraSensor:

    def __init__(self, clientID):
        # Connection ID to communicate with simulator
        self.clientID = clientID
        self.name = 'Vision_sensor'
        self.camera = 0
        # Get the Vision Sensor Handle 
        error,self.camera = sim.simxGetObjectHandle(self.clientID,self.name,sim.simx_opmode_oneshot_wait)       
        error, Resol_XY, image = sim.simxGetVisionSensorImage(self.clientID,self.camera,0,sim.simx_opmode_streaming)

    def get_SnapShot(self):
        # Get Resolution and Image in RGB scale from simxGetVisionSensorImage
        error, Resol_XY, image = sim.simxGetVisionSensorImage(self.clientID,self.camera,0,sim.simx_opmode_buffer)

        # Reshape the array to 3d matrix
        img = np.array(image, dtype = np.uint8)
        img.resize([Resol_XY[0],Resol_XY[1],3])
        # Flip the figure
        img = np.flip(img, 0)
        # Return the Resolution and the Image Matrix
        return Resol_XY, img