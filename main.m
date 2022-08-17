clc;clear;close all;
disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
    
if (clientID>-1)
    disp('Connected to remote API server');
    fis=load("wheel_fis.mat");
    simTime=300;
    Handle=ObjectHandle(clientID,sim);
    Simulation(clientID,sim,Handle,fis,simTime);
    sim.simxGetPingTime(clientID);
    sim.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
sim.delete();
    