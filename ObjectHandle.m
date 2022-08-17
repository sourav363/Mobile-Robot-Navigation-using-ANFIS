function Handle = ObjectHandle(clientID,sim)
    RobotHandle = zeros(6,1);
    SensorHandle = zeros(6,3);
    MotorHandle = zeros(6,2);
    TargetHandle = zeros(6,1);
   %%Getting Object Handles
   %Robot Handles
    [~,RobotHandle(1,:)]=sim.simxGetObjectHandle(clientID,'Robot1',sim.simx_opmode_blocking);
    [~,RobotHandle(2,:)]=sim.simxGetObjectHandle(clientID,'Robot2',sim.simx_opmode_blocking);
    [~,RobotHandle(3,:)]=sim.simxGetObjectHandle(clientID,'Robot3',sim.simx_opmode_blocking);
    [~,RobotHandle(4,:)]=sim.simxGetObjectHandle(clientID,'Robot4',sim.simx_opmode_blocking);
    [~,RobotHandle(5,:)]=sim.simxGetObjectHandle(clientID,'Robot5',sim.simx_opmode_blocking);
    [~,RobotHandle(6,:)]=sim.simxGetObjectHandle(clientID,'Robot6',sim.simx_opmode_blocking);
    
   %SensorHandles
    [~, SensorHandle(1,1)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_left#0', sim.simx_opmode_blocking);
    [~, SensorHandle(1,2)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_front#0', sim.simx_opmode_blocking);
    [~, SensorHandle(1,3)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_right#0', sim.simx_opmode_blocking);
    
    [~, SensorHandle(2,1)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_left#1', sim.simx_opmode_blocking);
    [~, SensorHandle(2,2)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_front#1', sim.simx_opmode_blocking);
    [~, SensorHandle(2,3)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_right#1', sim.simx_opmode_blocking);
    
    [~, SensorHandle(3,1)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_left#2', sim.simx_opmode_blocking);
    [~, SensorHandle(3,2)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_front#2', sim.simx_opmode_blocking);
    [~, SensorHandle(3,3)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_right#2', sim.simx_opmode_blocking);
    
    [~, SensorHandle(4,1)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_left#3', sim.simx_opmode_blocking);
    [~, SensorHandle(4,2)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_front#3', sim.simx_opmode_blocking);
    [~, SensorHandle(4,3)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_right#3', sim.simx_opmode_blocking);
    
    [~, SensorHandle(5,1)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_left#4', sim.simx_opmode_blocking);
    [~, SensorHandle(5,2)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_front#4', sim.simx_opmode_blocking);
    [~, SensorHandle(5,3)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_right#4', sim.simx_opmode_blocking);
    
    [~, SensorHandle(6,1)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_left#5', sim.simx_opmode_blocking);
    [~, SensorHandle(6,2)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_front#5', sim.simx_opmode_blocking);
    [~, SensorHandle(6,3)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_right#5', sim.simx_opmode_blocking);
    
   %MotorHandles
    [~,MotorHandle(1,1)]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor#0',sim.simx_opmode_blocking);
    [~,MotorHandle(1,2)]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor#0',sim.simx_opmode_blocking);
    
    [~,MotorHandle(2,1)]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor#1',sim.simx_opmode_blocking);
    [~,MotorHandle(2,2)]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor#1',sim.simx_opmode_blocking);
   
    [~,MotorHandle(3,1)]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor#2',sim.simx_opmode_blocking);
    [~,MotorHandle(3,2)]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor#2',sim.simx_opmode_blocking);

    [~,MotorHandle(4,1)]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor#3',sim.simx_opmode_blocking);
    [~,MotorHandle(4,2)]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor#3',sim.simx_opmode_blocking);

    [~,MotorHandle(5,1)]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor#4',sim.simx_opmode_blocking);
    [~,MotorHandle(5,2)]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor#4',sim.simx_opmode_blocking);

    [~,MotorHandle(6,1)]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor#5',sim.simx_opmode_blocking);
    [~,MotorHandle(6,2)]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor#5',sim.simx_opmode_blocking);
    
   %TargetHandles
    [~,TargetHandle(1,:)]=sim.simxGetObjectHandle(clientID,'dummy1',sim.simx_opmode_blocking);
    [~,TargetHandle(2,:)]=sim.simxGetObjectHandle(clientID,'dummy2',sim.simx_opmode_blocking);
    [~,TargetHandle(3,:)]=sim.simxGetObjectHandle(clientID,'dummy3',sim.simx_opmode_blocking);
    [~,TargetHandle(4,:)]=sim.simxGetObjectHandle(clientID,'dummy4',sim.simx_opmode_blocking);
    [~,TargetHandle(5,:)]=sim.simxGetObjectHandle(clientID,'dummy5',sim.simx_opmode_blocking);
    [~,TargetHandle(6,:)]=sim.simxGetObjectHandle(clientID,'dummy6',sim.simx_opmode_blocking);

    Handle=[RobotHandle, SensorHandle, MotorHandle, TargetHandle];
end

