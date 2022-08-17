function Simulation(clientID,sim,Handle,fis,simTime)
   
       %Object Handles
        RobotHandle = Handle(:,1);
        SensorHandle = Handle(:,2:4);
        MotorHandle = Handle(:,5:6);
        TargetHandle = Handle(:,7);
     
        %Plot preparation

        figure("Name",'Path Planning for Multiple Robots','WindowState','minimized','Color',[0.8 0.8 0.8]);
        subplot(3,5,[1 2 3 6 7 8 11 12 13])
        h1 = animatedline("Color",'b','LineWidth',2);
        h2 = animatedline("Color",'r','LineWidth',2);
        h3 = animatedline("Color",'g','LineWidth',2);
        h4 = animatedline("Color",'k','LineWidth',2);
        h5 = animatedline("Color",'y','LineWidth',2);
        h6 = animatedline("Color",'c','LineWidth',2);
        axis([-12.5,12.5,-12.5,12.5]);
        set(gca, 'XDir','reverse','YDir','reverse')
        xlabel('x');ylabel('y');title('Trajectory');
        box on
        
        %figure("Name",'velocity','WindowState','minimized');
        subplot(3,5,[4 5])
        h7 = animatedline("Color",'b','LineWidth',0.5);
        h8 = animatedline("Color",'r','LineWidth',0.5);
        h9 = animatedline("Color",'g','LineWidth',0.5);
        h10 = animatedline("Color",'k','LineWidth',0.5);
        h11 = animatedline("Color",'y','LineWidth',0.5);
        h12 = animatedline("Color",'c','LineWidth',0.5);
        xlabel('time(s)');ylabel('Velocity(m/s)');title('Velocty vs. Time');
        box on
        
        %figure("Name",'SA','WindowState','minimized');
        subplot(3,5,[9 10])
        h13 = animatedline("Color",'b','LineWidth',0.5);
        h14 = animatedline("Color",'r','LineWidth',0.5);
        h15 = animatedline("Color",'g','LineWidth',0.5);
        h16 = animatedline("Color",'k','LineWidth',0.5);
        h17 = animatedline("Color",'y','LineWidth',0.5);
        h18 = animatedline("Color",'c','LineWidth',0.5);
        box on
        xlabel('time(s)');ylabel('Steering Angle(degree)');title('Steering Angle vs Time');
        
        %figure("Name",'distance','WindowState','minimized');
        subplot(3,5,[14 15])
        h19 = animatedline("Color",'b','LineWidth',0.5);
        h20 = animatedline("Color",'r','LineWidth',0.5);
        h21 = animatedline("Color",'g','LineWidth',0.5);
        h22 = animatedline("Color",'k','LineWidth',0.5);
        h23 = animatedline("Color",'y','LineWidth',0.5);
        h24 = animatedline("Color",'c','LineWidth',0.5);
        box on
        xlabel('time(s)');ylabel('Distance(m)');title('Distance from Target vs Time');
        
        %Simulation preparation      
        RobotPosition=zeros(6,3);
        TargetPositionRobotFrame=zeros(6,3);
        SensorDetectionState=zeros(6,3);
        SensorDetectedPoint=zeros(3,3,6);
        OD=zeros(6,3);
        Distance=zeros(6,1);
        AVel=zeros(6,2);
        Vel=zeros(6,3); 
        HA=zeros(6,1);
        itr = 0;
        tol = 0.1;
        elapsedTime = 0;
        tic;

        while elapsedTime<=simTime  
            itr=itr+1;
            if itr==1
                opmode = sim.simx_opmode_streaming;
            else
                opmode = sim.simx_opmode_buffer;
            end
               
            for i=1:6
                
                %Get Position data
                [~,RobotPosition(i,:)] = sim.simxGetObjectPosition(clientID,RobotHandle(i,:),-1,opmode);
                [~,TargetPositionRobotFrame(i,:)]= sim.simxGetObjectPosition(clientID,TargetHandle(i,:),RobotHandle(i,:),opmode);
                Distance(i,:)=norm(norm(TargetPositionRobotFrame(i,:)));
                [~,Vel(i,:)]=sim.simxGetObjectVelocity(clientID,RobotHandle(i,:),opmode);
                %Get Sensor data
                [~,SensorDetectionState(i,1),SensorDetectedPoint(1,:,i),~,~] = sim.simxReadProximitySensor(clientID,SensorHandle(i,1),opmode);
                [~,SensorDetectionState(i,2),SensorDetectedPoint(2,:,i),~,~] = sim.simxReadProximitySensor(clientID,SensorHandle(i,2),opmode);
                [~,SensorDetectionState(i,3),SensorDetectedPoint(3,:,i),~,~] = sim.simxReadProximitySensor(clientID,SensorHandle(i,3),opmode);
                
                if Distance(i,:)>=tol
                     if SensorDetectionState(i,1) == 0 && SensorDetectionState(i,2) == 0 && SensorDetectionState(i,3) == 0   %No Obstacle
                        %Heading Angle Calculation
                        HA(i,:)=-atan2d(TargetPositionRobotFrame(i,2),TargetPositionRobotFrame(i,1));
                        
                        %ANFIS No Obstacle
                        AVel(i,1) = evalfis(fis.lw_noObstacle,HA(i,:));
                        AVel(i,2) = evalfis(fis.rw_noObstacle,HA(i,:));
                        if Distance(i,:)>1
                        %setting joint velocities
                        sim.simxSetJointTargetVelocity(clientID,MotorHandle(i,1),0.05*AVel(i,1),sim.simx_opmode_oneshot);
                        sim.simxSetJointTargetVelocity(clientID,MotorHandle(i,2),0.05*AVel(i,2),sim.simx_opmode_oneshot);
                        else
                        sim.simxSetJointTargetVelocity(clientID,MotorHandle(i,1),Distance(i,:)*0.05*AVel(i,1),sim.simx_opmode_oneshot);
                        sim.simxSetJointTargetVelocity(clientID,MotorHandle(i,2),Distance(i,:)*0.05*AVel(i,2),sim.simx_opmode_oneshot);
                        end
                    else                                                                                            %Obstacle
                        OD(i,:) = ObstacleDistance(SensorDetectionState(i,:),SensorDetectedPoint(:,:,i));
                        %ANFIS Obstacle
                        AVel(i,1) = evalfis(fis.lw_Obstacle, OD(i,:));
                        AVel(i,2) = evalfis(fis.rw_Obstacle, OD(i,:));
                    
                        %setting joint velocities
                        sim.simxSetJointTargetVelocity(clientID,MotorHandle(i,1),0.05*AVel(i,1),sim.simx_opmode_oneshot);
                        sim.simxSetJointTargetVelocity(clientID,MotorHandle(i,2),0.05*AVel(i,2),sim.simx_opmode_oneshot);
                    end
                else
                    %stop conditions
                    sim.simxSetJointTargetVelocity(clientID,MotorHandle(i,1),0,sim.simx_opmode_oneshot);
                    sim.simxSetJointTargetVelocity(clientID,MotorHandle(i,2),0,sim.simx_opmode_oneshot);
                end    

            end        
            if mod(itr,30)==0
                %plot
                addpoints(h1,double(RobotPosition(1,1)),double(RobotPosition(1,2)));  
                addpoints(h2,double(RobotPosition(2,1)),double(RobotPosition(2,2)));
                addpoints(h3,double(RobotPosition(3,1)),double(RobotPosition(3,2)));
                addpoints(h4,double(RobotPosition(4,1)),double(RobotPosition(4,2)));
                addpoints(h5,double(RobotPosition(5,1)),double(RobotPosition(5,2)));
                addpoints(h6,double(RobotPosition(6,1)),double(RobotPosition(6,2)));

                addpoints(h7,elapsedTime,double(norm(Vel(1,1))));  
                addpoints(h8,elapsedTime,double(norm(Vel(2,1))));
                addpoints(h9,elapsedTime,double(norm(Vel(3,1))));
                addpoints(h10,elapsedTime,double(norm(Vel(4,1))));
                addpoints(h11,elapsedTime,double(norm(Vel(5,1))));
                addpoints(h12,elapsedTime,double(norm(Vel(6,1))));

                addpoints(h13,elapsedTime,double(-HA(1,:)));  
                addpoints(h14,elapsedTime,double(-HA(2,:)));
                addpoints(h15,elapsedTime,double(-HA(3,:)));
                addpoints(h16,elapsedTime,double(-HA(4,:)));
                addpoints(h17,elapsedTime,double(-HA(5,:)));
                addpoints(h18,elapsedTime,double(-HA(6,:)));

                addpoints(h19,elapsedTime,double(Distance(1,:)));  
                addpoints(h20,elapsedTime,double(Distance(2,:)));
                addpoints(h21,elapsedTime,double(Distance(3,:)));
                addpoints(h22,elapsedTime,double(Distance(4,:)));
                addpoints(h23,elapsedTime,double(Distance(5,:)));
                addpoints(h24,elapsedTime,double(Distance(6,:)));
                
                drawnow limitrate;
            end
            elapsedTime = toc;   
        end

