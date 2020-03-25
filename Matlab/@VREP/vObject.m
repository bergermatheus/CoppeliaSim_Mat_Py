function vObject(vrep,String)
      [erro,Tag] = vrep.RemApi.simxGetObjectHandle(vrep.clientID,String,vrep.RemApi.simx_opmode_oneshot_wait);
      if (erro > 0)
          disp('Object not found!')
      else
          [erro,~] = vrep.RemApi.simxGetObjectPosition(vrep.clientID,Tag,-1,vrep.RemApi.simx_opmode_streaming);
      end
      aux = {{String}, Tag};
      vrep.Objects = [vrep.Objects ; aux ];
end