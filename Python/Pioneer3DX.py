import sim
import numpy as np

class Pioneer3DX:

    def __init__(self, clientID):
        self.clientID = clientID
        self.pioneer3DX_array = [None] * 19
        self.position_coordX = [None] * 3
        self.position_coordXc = [None] * 3
        self.orientation = None
        self.velocity = [None]* 2
        self.ultrasonic = np.zeros((16, 3))
        self.name = "Pioneer_p3dx"
        self.left_motor = 'Pioneer_p3dx_leftMotor'
        self.right_motor = 'Pioneer_p3dx_rightMotor'
        self.ultrasonic_sensors = "Pioneer_p3dx_ultrasonicSensor"

        ### Load Mobile Robot Pioneer parameters
        # self.pioneer3DX_array[0] represents the entire mobile robot block
        error,self.pioneer3DX_array[0] = sim.simxGetObjectHandle(self.clientID,self.name,sim.simx_opmode_blocking)
        # self.pioneer3DX_array[1] represents only the left motor 
        error,self.pioneer3DX_array[1] = sim.simxGetObjectHandle(self.clientID,self.left_motor,sim.simx_opmode_blocking)
        # self.pioneer3DX_array[2] represents only the right motor
        error,self.pioneer3DX_array[2] = sim.simxGetObjectHandle(self.clientID,self.right_motor,sim.simx_opmode_blocking)

        # self.pioneer3DX_array[3:18] represents the 16 ultrasonic sensors
        num = 1
        while num < 17:
            error,self.pioneer3DX_array[2+num] = sim.simxGetObjectHandle(self.clientID,self.ultrasonic_sensors+str(num),sim.simx_opmode_blocking)
            error, DetectionState, Points ,detectedObjectHandle, Vector = sim.simxReadProximitySensor(self.clientID,self.pioneer3DX_array[2+num],sim.simx_opmode_streaming)
            num+=1


        error, linearVelocity, angularVelocity = sim.simxGetObjectVelocity(self.clientID,self.pioneer3DX_array[0], sim.simx_opmode_streaming)
        error, position = sim.simxGetObjectPosition(self.clientID,self.pioneer3DX_array[0],-1, sim.simx_opmode_streaming)
        error, angle = sim.simxGetObjectOrientation(self.clientID,self.pioneer3DX_array[0],-1,sim.simx_opmode_streaming)
        print("Pioneer 3DX loaded")
        

    def send_ControlSignals(self,Ud):
        # left_speed = (v - e w)/R 
        # right_speed = (v + e w)/R
        # Ud[0] is the linear velocity given as parameter
        # Ud[1] is the angular velocity given as parameter
        # e is half of the distance between the left and right wheels
        # R is the radius of the wheels
        R = 0.0925
        e = 0.17

        left_speed  = (Ud[0] - e*Ud[1])/R
        right_speed = (Ud[0] + e*Ud[1])/R

        # Send Control Signal to each motor (first left and second right)
        error = sim.simxSetJointTargetVelocity(self.clientID,self.pioneer3DX_array[1],left_speed,sim.simx_opmode_streaming)
        error = sim.simxSetJointTargetVelocity(self.clientID,self.pioneer3DX_array[2],right_speed,sim.simx_opmode_streaming)



    def get_PositionData(self):
        ## Pioneer Velocity
        error, linearVelocity, angularVelocity = sim.simxGetObjectVelocity(self.clientID,self.pioneer3DX_array[0], sim.simx_opmode_buffer)
        ## Pioneer Position
        error, self.position_coordXc = sim.simxGetObjectPosition(self.clientID,self.pioneer3DX_array[0],-1, sim.simx_opmode_buffer)
        ## Pioneer Orientation (alpha, beta e gama)
        error, angle = sim.simxGetObjectOrientation(self.clientID,self.pioneer3DX_array[0],-1,sim.simx_opmode_buffer)
        self.orientation = angle[2]
        # Linear Transform to find the Control Point
        A = np.array([np.cos(angle[2]), np.sin(angle[2]), 0])
        self.position_coordX = self.position_coordXc + 0.15*A

        # @remove get functions normally return values(the position and orientation)
        return self.position_coordX, self.orientation
        
        

    def get_UltrasonicData(self):
        # Get ultrasonic data from Pioneer
        num = 1
        while num < 17:
            error, DetectionState, self.ultrasonic[num-1,:] ,detectedObjectHandle, Vector = sim.simxReadProximitySensor(self.clientID,self.pioneer3DX_array[2+num],sim.simx_opmode_buffer)
            self.ultrasonic[num-1,2] = np.linalg.norm(self.ultrasonic[num-1,0:2])*DetectionState
            num+=1
        print(self.ultrasonic[:,2])

    # Define Direct Kinematic (for differential drive robot)
    def get_K_diff_drive_robot(self, X_currRealOrientation):
        # K = [[cos(theta)  -0.15*sin(theta)
        #       sin(theta)   0.15*cos(theta)]]
        K = np.array([[np.cos(X_currRealOrientation),-0.15*np.sin(X_currRealOrientation)],[np.sin(X_currRealOrientation),0.15*np.cos(X_currRealOrientation)]])
        return K

    # Define controller signal
    def lyapunov_controller_signal(self, kinematic, X_diff, Xtil):
        # Lyapunov Controller: Ud = K^-1*(0.4*X_diff + 0.7*tanh(0.5Xtil))
        # Inverse kinematic K^-1
        a = np.linalg.inv(kinematic)
        b = 0.3*X_diff.transpose() + 1*np.tanh(0.8*Xtil)
        Ud = np.dot(a,b)
        return Ud

    # Define trajectory
    def get_curr_desired_point_CIRCLE(self, tStep):
        # Parameters of the circle
        r = 1.5
        T = 40.0
        w = 1/T
        return [r * np.cos(2*np.pi*w * tStep), r * np.sin(2*np.pi*w * tStep)]