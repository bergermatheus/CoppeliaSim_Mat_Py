import sim

class Pioneer3DX:

    def __init__(self, clientID):
        self.clientID = clientID
        self.pioneer3DX_array = [None] * 19
        self.name = "Pioneer_p3dx"
        self.left_motor = 'Pioneer_p3dx_leftMotor'
        self.right_motor = 'Pioneer_p3dx_rightMotor'
        self.ultrasonic_sensors = "Pioneer_p3dx_ultrasonicSensor"
        
        ### Load Mobile Robot Pioneer parameters
        # self.pioneer3DX_array[0] represents the entire mobile robot block
        # self.pioneer3DX_array[1] represents only the left motor 
        # self.pioneer3DX_array[2] represents only the right motor
        error,self.pioneer3DX_array[0] = sim.simxGetObjectHandle(self.clientID,self.name,sim.simx_opmode_blocking)
        error,self.pioneer3DX_array[1] = sim.simxGetObjectHandle(self.clientID,self.left_motor,sim.simx_opmode_blocking)
        error,self.pioneer3DX_array[2] = sim.simxGetObjectHandle(self.clientID,self.right_motor,sim.simx_opmode_blocking)

        # self.pioneer3DX_array[3:18] represents the 16 ultrasonic sensors
        num = 1
        while num < 17:
            error,self.pioneer3DX_array[2+num] = sim.simxGetObjectHandle(self.clientID,self.ultrasonic_sensors+str(num),sim.simx_opmode_blocking)
            num+=1

        print("Pioneer 3DX loaded")
        #return self.pioneer3DX_array

    
    #def get_PositionData(self):


    #def get_UltrasonicData(self):

    