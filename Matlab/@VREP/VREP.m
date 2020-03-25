classdef VREP < handle
    
    properties
        RemApi;         % V-Rep library remote Api
        clientID;       % Indentificator adress communication
        Pioneer = zeros(19,1);    % Robot Tags 1-Pionner, 2-Left Motor, 3 Right Motor, 4:19-Ultrasonic
        Objects = {};   % Objects Tags
    end
    % In a methods block, set the method attributes
    % and add the function signature
    methods
        %% Constructor Class
        function vrep = VREP
            vrep.RemApi = remApi('remoteApi');
        end
        
        
        %% Connection fuctions
        % ==================================================
        vConnect(vrep);     % connect Matlab and V-REP
        vDisconnect(vrep);  % disconnect Matlab and V-REP
        % ==================================================
     
        %% Other functions
        % ==================================================        
        % Handle Objects
        vHandle(vrep,String,Index); % Pioneer Object
        vObject(vrep,String);       % Other Objects
        
        % Laser Scanner
        [Map] = vGetLaserData(vrep,Index);
        
        % Data Request
        % Get Data from Pioneer
        [Xc,X,U] = vGetSensorData(vrep,Index); % Xc - Center Position, X - Control Position, U - Velocity 
        % Get Data from Objects
        [Xc,O] = vGetObjPosition(vrep,String); % Xc - Center Position, O - Orientation
        
        % Send Command
        vSendControlSignals(vrep,Ud,Index); % Send Control Signals to V-REP
   
    end
end