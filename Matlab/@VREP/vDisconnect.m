%% Disconnect Matlab to V-Rep via client service

% Now send some data to V-REP in a non-blocking fashion:
function vDisconnect(vrep)
    vrep.RemApi.simxAddStatusbarMessage(vrep.clientID,'Disconnected to Matlab',vrep.RemApi.simx_opmode_oneshot);

    % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.RemApi.simxGetPingTime(vrep.clientID);

    % Now close the connection to V-REP:    
    vrep.RemApi.simxFinish(vrep.clientID);
    vrep.RemApi.delete(); % call the destructor!

    disp('Communication ended');
end