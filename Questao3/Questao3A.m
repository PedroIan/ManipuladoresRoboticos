% Make sure to have the server side running in CoppeliaSim:
% in a child script of a CoppeliaSim scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if (clientID >- 1)
    disp('Connected to remote API server');

    h = [0 0 0 0 0 0 0];
    
    joint_position = [pi/2 pi/2 0 0 0 pi/2 0];
    
    [r, h(1)] = sim.simxGetObjectHandle(clientID, 'joint_7', sim.simx_opmode_blocking);
    [r, h(2)] = sim.simxGetObjectHandle(clientID, 'joint_1', sim.simx_opmode_blocking);
    [r, h(3)] = sim.simxGetObjectHandle(clientID, 'joint_2', sim.simx_opmode_blocking);
    [r, h(4)] = sim.simxGetObjectHandle(clientID, 'joint_3', sim.simx_opmode_blocking);
    [r, h(5)] = sim.simxGetObjectHandle(clientID, 'joint_4', sim.simx_opmode_blocking);
    [r, h(6)] = sim.simxGetObjectHandle(clientID, 'joint_5', sim.simx_opmode_blocking);
    [r, h(7)] = sim.simxGetObjectHandle(clientID, 'joint_6', sim.simx_opmode_blocking);
    
    
    for i=1:7
        sim.simxSetJointTargetPosition(clientID, h(i), joint_position(i), sim.simx_opmode_streaming)
    end
    
    [tete, position] = sim.simxGetJointPosition(clientID, h(2), sim.simx_opmode_streaming)
    pause(10);
    %sim.simxGetJointMatrix(clientID)
else
    disp('Failed connecting to remote API server');
end

sim.delete(); % call the destructor!

disp('Program ended');
