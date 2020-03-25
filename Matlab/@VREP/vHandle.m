%% Object Handles from V-Rep
%[erro,handles, intData, floatData,stringData]= p3dx.vrep.simxGetObjectGroupData(p3dx.clientID,p3dx.vrep.sim_appobj_object_type,0,p3dx.vrep.simx_opmode_blocking )
function vHandle(vrep,String,Index)
    
    switch nargin
        case 1 % If user didn't write Robot String name
            prompt = {'Necessary to enter the robot name. Example:'};
            title = 'Warning';
            definput = {'Pioneer_p3dx'};
            opts.Interpreter = 'tex';
            answer = inputdlg(prompt,title,[1 40],definput,opts);

            vrep.vHandle(answer{1});
        case 2 % If user wrote Robot String name
            %% Pioneer Handle (array position 1)

            [erro,vrep.Pioneer(1)] = vrep.RemApi.simxGetObjectHandle(vrep.clientID, String, vrep.RemApi.simx_opmode_oneshot_wait);
            if (erro > 0)
                disp('Object Pioneer 3DX not found!')
            else
                [erro,~, ~] = vrep.RemApi.simxGetObjectVelocity(vrep.clientID,vrep.Pioneer(1),vrep.RemApi.simx_opmode_streaming);
                [erro,~]    = vrep.RemApi.simxGetObjectPosition(vrep.clientID,vrep.Pioneer(1),-1,vrep.RemApi.simx_opmode_streaming);
                [erro,~]    = vrep.RemApi.simxGetObjectOrientation(vrep.clientID,vrep.Pioneer(1),-1,vrep.RemApi.simx_opmode_streaming);
            end

            

            %% Pioneer Motor Handle (array position 2)
            Motor = [String '_leftMotor'];
            [erro,vrep.Pioneer(2)] = vrep.RemApi.simxGetObjectHandle(vrep.clientID, Motor ,vrep.RemApi.simx_opmode_oneshot_wait);
            if (erro > 0)
                disp('Object left Motor not found!')
            else
                [erro, ~]= vrep.RemApi.simxGetJointForce(vrep.clientID,vrep.Pioneer(2), vrep.RemApi.simx_opmode_streaming);
            end
            
            %% Pioneer Motor Handle (array position 3)
            Motor = [String '_rightMotor'];
            [erro,vrep.Pioneer(3)] = vrep.RemApi.simxGetObjectHandle(vrep.clientID, Motor, vrep.RemApi.simx_opmode_oneshot_wait);
            if (erro > 0)
                disp('Object right Motor not found!')
            else
                [erro, ~]= vrep.RemApi.simxGetJointForce(vrep.clientID, vrep.Pioneer(3), vrep.RemApi.simx_opmode_streaming);
            end
                        

            %% Pioneer Ultrasonic Handle (array position 4-19)
            Sensor = [String '_ultrasonicSensor'];
            for i=1:16
                ultrasonic_index = [Sensor num2str(i)];
                [erro, vrep.Pioneer(3+i)] = vrep.RemApi.simxGetObjectHandle(vrep.clientID, ultrasonic_index, vrep.RemApi.simx_opmode_oneshot_wait);
                if erro > 0
                    msg = ['Ultrasonic ' num2str(i) ' not found!'];
                    disp(msg)
                else
                    [erro, state, coord, ~, ~] = vrep.RemApi.simxReadProximitySensor(vrep.clientID, vrep.Pioneer(i+3),vrep.RemApi.simx_opmode_streaming);
                end
            end
            
%          
           %% Get Laser Data
           [erro,~] = vrep.RemApi.simxGetStringSignal(vrep.clientID,'communication',vrep.RemApi.simx_opmode_streaming);
% 
    
    case 3  % If user wrote Robot String name and Index Robot
        
        
        Name = [String '#' Index];
%         
        [erro,aux] = vrep.RemApi.simxGetObjectHandle(vrep.clientID,Name,vrep.RemApi.simx_opmode_oneshot_wait);
        if (erro > 0)
            disp('Object Pioneer 3DX not found!')
        else
            %% Pioneer Handle (array position 1)
            vrep.Pioneer = [vrep.Pioneer zeros(19,1)];
            vrep.Pioneer(1,end) = aux;
            
            [erro,~, ~] = vrep.RemApi.simxGetObjectVelocity(vrep.clientID,vrep.Pioneer(1,end),vrep.RemApi.simx_opmode_streaming);
            [erro,~] =    vrep.RemApi.simxGetObjectPosition(vrep.clientID,vrep.Pioneer(1,end),-1,vrep.RemApi.simx_opmode_streaming);
            [erro,~] =    vrep.RemApi.simxGetObjectOrientation(vrep.clientID,vrep.Pioneer(1,end),-1,vrep.RemApi.simx_opmode_streaming);
        end
        
        %% Pioneer Motor Handle (array position 2)
        % Left Motor
        Motor = [String '_leftMotor#' Index];
        [erro,aux] = vrep.RemApi.simxGetObjectHandle(vrep.clientID, Motor ,vrep.RemApi.simx_opmode_oneshot_wait);
        if (erro > 0)
            disp('Left Motor not found!')
        else
            vrep.Pioneer(2,end) = aux;
            [erro, ~] = vrep.RemApi.simxGetJointForce(vrep.clientID,vrep.Pioneer(2,end), vrep.RemApi.simx_opmode_streaming);
        end
        
    
        %% Pioneer Motor Handle (array position 3)
        %Right Motor
        Motor = [String '_rightMotor#' Index];
        
        [erro,aux] = vrep.RemApi.simxGetObjectHandle(vrep.clientID, Motor ,vrep.RemApi.simx_opmode_oneshot_wait);
        if (erro > 0)
            disp('Right Motor not found!')
        else
            vrep.Pioneer(3,end) = aux;
            [erro, ~]= vrep.RemApi.simxGetJointForce(vrep.clientID, vrep.Pioneer(3,end), vrep.RemApi.simx_opmode_streaming);
        end

        


        %% Pioneer Ultrasonic Handle (array position 4-19)
            Sensor = [String '_ultrasonicSensor'];
            for i=1:16
                ultrasonic_index = [Sensor num2str(i) '#' Index];
                [erro,aux] = vrep.RemApi.simxGetObjectHandle(vrep.clientID, ultrasonic_index, vrep.RemApi.simx_opmode_oneshot_wait);
                if erro > 0
                    msg = ['Ultrasonic ' num2str(i) ' not found!'];
                    disp(msg)
                else
                    vrep.Pioneer(3+i,end) = aux;
                    [erro, state, coord, ~, ~] = vrep.RemApi.simxReadProximitySensor(vrep.clientID, vrep.Pioneer(i+3,end),vrep.RemApi.simx_opmode_streaming);
                
                end
            end
        
        
    end
    

end