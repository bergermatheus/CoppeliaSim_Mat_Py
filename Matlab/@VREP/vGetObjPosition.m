
%% vGetObjPosition is responsible to get position X,Y,Z and orientation from
% objects which do not move 
function [Xc,O] = vGetObjPosition(vrep,String)
    %% Searching for the String Name of the Object
    for i=1:length(vrep.Objects(:,1))
        if String == string(vrep.Objects{i,1}) % if found, break
            index = i;
            break;
        end
    end
    %% Get Position and Orientation from VREP
    [erro,Xc] = vrep.RemApi.simxGetObjectPosition(vrep.clientID,vrep.Objects{index,2},-1,vrep.RemApi.simx_opmode_buffer);
    [erro,O] = vrep.RemApi.simxGetObjectOrientation(vrep.clientID,vrep.Objects{index,2},-1,vrep.RemApi.simx_opmode_buffer);
end