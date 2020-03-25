function vSendControlSignals(vrep,Ud,Index)
    % left_speed = (v - e w)/R 
    % right_speed = (v + e w)/R
    % v is the linear velocity given as parameter
    % w is the angular velocity given as parameter
    % e is half of the distance between the left and right wheels
    % R is the radius of the wheels
    R= 0.0925; %raio da roda do Pioneer
    e = 0.17; % distancia do centro até a roda
    
    left_speed  = (Ud(1) - e*Ud(2))/R;
    right_speed = (Ud(1) + e*Ud(2))/R;
    
    erro = vrep.RemApi.simxSetJointTargetVelocity(vrep.clientID, vrep.Pioneer(2,Index), left_speed, vrep.RemApi.simx_opmode_streaming);
    erro = vrep.RemApi.simxSetJointTargetVelocity(vrep.clientID, vrep.Pioneer(3,Index), right_speed, vrep.RemApi.simx_opmode_streaming);
end