%% This function gets information about Position and Velocity
function [Xc,X, U] = vGetSensorData(vrep,Index)
    Xc = zeros(2,1); X = zeros(2,1); U = zeros(2,1);
    %% Pioneer Position
    [erro,coordinateXYZ]= vrep.RemApi.simxGetObjectPosition(vrep.clientID,vrep.Pioneer(1,Index),-1,vrep.RemApi.simx_opmode_buffer);

    Xc = coordinateXYZ([1 2])';
    
    %% Pioneer Orientation (alpha, beta e gama)
    [erro,orientation] = vrep.RemApi.simxGetObjectOrientation(vrep.clientID,vrep.Pioneer(1,Index),-1,vrep.RemApi.simx_opmode_buffer);
    
    % Linear Transform to find the Control Point
    X = Xc + [0.15*cos(orientation(3)); 0.15*sin(orientation(3))]; 

    

    %% Pioneer Linear and Angular Velocity
    [erro,LinearVelocity, AngularVelocity]= vrep.RemApi.simxGetObjectVelocity(vrep.clientID,vrep.Pioneer(1,Index),vrep.RemApi.simx_opmode_buffer);


    theta = atan2(LinearVelocity(2),LinearVelocity(1));
    Psi = orientation(3);

    if theta*Psi <0
        U(1) = -sqrt(LinearVelocity(1)^2+LinearVelocity(2)^2 +LinearVelocity(3)^2);
    else
        U(1) = sqrt(LinearVelocity(1)^2+LinearVelocity(2)^2 +LinearVelocity(3)^2);
    end
    
    U(2) = AngularVelocity(3);
end