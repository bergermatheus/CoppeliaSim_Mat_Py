% Make sure to have the server side running in V-REP: 
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999) or simxStart('127.0.0.2',19997)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

%% Connect Matlab to V-Rep via client service
function vConnect(vrep)
    disp('Program trying to communicate...');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep.RemApi.simxFinish(-1); % just in case, close all opened connections
    vrep.clientID = vrep.RemApi.simxStart('127.0.0.2',19997,true,true,5000,5);

    if (vrep.clientID>-1)
        disp('Connected to remote API server');
        vrep.RemApi.simxAddStatusbarMessage(vrep.clientID,'Connected to Matlab',vrep.RemApi.simx_opmode_oneshot);
        disp('Successful communication');
        pause(2);

        % Now try to retrieve data in a blocking fashion (i.e. a service call):
        [erro,objs]= vrep.RemApi.simxGetObjects(vrep.clientID,vrep.RemApi.sim_handle_all,vrep.RemApi.simx_opmode_blocking);
        if (erro == vrep.RemApi.simx_return_ok)
            fprintf('Number of objects in the V-Rep scene: %d\n',length(objs));
        else
            fprintf('Remote API function call returned with error code: %d\n',erro);
        end

    else
        disp('Failed connecting to remote API server');
        vrep.RemApi.delete(); % call the destructor!
    end

end