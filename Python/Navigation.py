## Funcions for Navigation
import numpy as np
# Define Avoidance Strategy
## Potencial Field avoidance obstacles function
def avoidance_Potencial_Field(currLaserDataX,currLaserDataY,X_currRealPos,X_Desired):
    # repulsion force
    Force_rep = np.zeros((2,1))
    # resultant force
    Force_result = np.zeros((2,1))
    Repulse_vector = np.zeros((2,1))
    Atract_vector = np.zeros((2,1))
    #constant repulsion
    Fk = 0.004
    #constant of attraction
    Fc = 0.4  
    flag_avoid = False
    #Distance from Robot to obstacles
    for i in range(0,len(currLaserDataX)):
        Distance  = np.hypot(currLaserDataX[i]-X_currRealPos[0],currLaserDataY[i]-X_currRealPos[1])
        
        #if the distance is shorter than 0.9 meter
        # Calculate the repulsive force
        if Distance <= 0.9:
            Force_rep[0] = -(Fk/Distance**2)*(currLaserDataX[i]-X_currRealPos[0])/Distance
            Force_rep[1] = -(Fk/Distance**2)*(currLaserDataY[i]-X_currRealPos[1])/Distance
            Repulse_vector = Repulse_vector + Force_rep
            flag_avoid = True
    
    ## Then calculate the atractive force as well
    if flag_avoid:
        # Positive Force       
        Dist_Robot_Goal = np.hypot(X_Desired[0]-X_currRealPos[0],X_Desired[1]-X_currRealPos[1])
        Force_atrac =Fc*(X_Desired-X_currRealPos[0:2])/Dist_Robot_Goal
        Force_atrac = np.array([Force_atrac])
        # Finally sum the vector to find the resultant force
        Force_result = Repulse_vector+Force_atrac.transpose()
    return flag_avoid, Force_result