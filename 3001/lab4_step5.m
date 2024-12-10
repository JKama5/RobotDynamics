%% config
travelTime = 25;
travelTimeMS = travelTime*1000;
figure;
point1 = [80 -75 40 300]; % [x y z alpha]
point2 = [190, 0, 290, 350];
point3 = [100 65 190 340];
points = [point1; point2; point3; point1];
pointsOrigin = [281.4 0 224.3254 360]; % [x y z alpha] origin

% Initialize coefficient variables
t0 = 0;
tf = travelTime;
v0 = 0;
vf = 0; %
a0 = 0;
af = 0;

numTimeSteps = 9; % TEMPORARY

robot = Robot(); % create robot instance
TP= Traj_Planner(); % create trajectory planner instance
M = Model(false);
robot.writeMotorState(true); % Write position mode
robot.set_joint_vars([0 0 0 0],3000); % send to home position
pause(3);

joints1 = robot.step2ik(point1(1:3),point1(4)); % in joint space
joints2 = robot.step2ik(point2(1:3),point2(4));
joints3 = robot.step2ik(point3(1:3),point3(4));
joints = [joints1; joints2; joints3; joints1];

%% Run
coefficents=zeros(6,4);
data=[];
for point=1:4
    coefficents(:,point)=TP.quintic_traj(0,travelTime,0,0,0,0,pointsOrigin(1,point),points(1,point));
end
%figure;
%data(:,:,1)=robot.run_trajectory(coefficents,travelTime,true);
robot.run_trajectoryVel(coefficents,travelTime);

for i = 1:size(points, 1)-1
    p0 = points(i, :);
    pf = points(i+1, :);
    for dim = 1:3 % For x, y and z dimensions
        coefficients_all_segments(:, dim, i) = TP.quintic_traj(0, travelTime, 0, 0, 0, 0, p0(dim), pf(dim));
    end
end
% 
for i=2:size(points,1)
    for point=1:4
        coefficents(:,point)=TP.quintic_traj(0,travelTime,0,0,0,0,points(i-1,point),points(i,point));
        %TP.plotCubicTrajWithCoefficents(coefficents(:,joint),0,travelTime,1)
    end
    robot.run_trajectoryVel(coefficents,travelTime); % blocking
    
    %Code to handle sizes of data collected from each side of the triangle
    % not matching 
%     if (size(data,2)>size(temp,2))
%         data=data(:,1:size(temp,2));
%     elseif (size(data,2)<size(temp,2))
%         temp=temp(:,1:size(data,2));
%     end
%     data(:,:,i)=temp;
end


% %% Plotting
% Pos=[];
% Time=[data(1,:,1) travelTime+data(1,:,2) 2*travelTime+data(1,:,3)];
% AllData=[data(2:end,:,1) data(2:end,:,2) data(2:end,:,3)];
% Angles=AllData(1:4,:);
% for i=1:size(Angles,2)
%     Pos(i,:)=(M.fk(Angles(1:4,i)));
% end
% 
% Velocities=AllData(5:8,:);


% JS=figure;
% set(JS, 'Name', 'Step 6 Joint Space')
% subplot(1,3,1);
% plot(Time,Angles);
% legend('θ1','θ2','θ3','θ4')
% title('Position vs Time')
% xlabel('Time (S)')
% ylabel('Position (degrees)')
% 
% 
% 
% d2ydt2=diff(Velocities');
% dt=diff(Time');
% for (i=1:size(d2ydt2,1))
%     d2ydt2(i,:)=d2ydt2(i,:)./dt(i);
% end
% 
% subplot(1,3,3)
% plot(Time(2:end),d2ydt2)
% legend('θ1"','θ2"','θ3"','θ4"')
% title('Angular Acceleration vs Time')
% xlabel('Time (S)')
% ylabel('Angular Velocity (degrees/s^2)')
% 
% TS=figure;subplot(1,3,2)
% plot(Time,Velocities)
% legend('X','Y','Z')
% title('Linear Velocity vs Time')
% xlabel('Time (S)')
% ylabel('Linear Velocity (mm/s)')
% set(TS,'Name','Step 6 Task Space')
% subplot(1,3,1)
% plot(Time,Pos);
% legend('X','Y','Z')
% title('End Effector Position vs Time')
% xlabel('Time (S)')
% ylabel('Position (mm)')
% 
% dpdt=diff(Pos);
% for (i=1:size(Time,2)-1)
%     d2ydt2(i,:)=d2ydt2(i,:)./dt(i);
% end
% subplot(1,3,2)
% plot(Time(1:end-1),dpdt)
% legend('dX/dt','dY/dt','dZ/dt')
% title('End Effector Velocity vs Time')
% xlabel('Time (S)')
% ylabel('Velocity (mm/s)')
% 
% d2pdt2=diff(dpdt);
% for (i=1:size(Time,2)-2)
%     d2pdt2(i,:)=d2pdt2(i,:)./dt(i);
% end
% subplot(1,3,3)
% plot(Time(1:end-2),d2pdt2)
% legend('X"','Y"','Z"')
% title('End Effector Accelerration vs Time')
% xlabel('Time (S)')
% ylabel('Position (mm/s^2)')
% 
% figure;
% X=Pos(:,1)';
% Y=Pos(:,2)';
% Z=Pos(:,3)';
% plot3(X,Y,Z)
% title('3D Path of End Effector in Task Space')
% xlabel('X Position (mm)')
% ylabel('Y Position (mm)')
% zlabel('Z Position (mm)')
figure;
plot3_init = plot3([], [], [], 'b.'); % Initialize ee pos plot
quiver_init = quiver3(0, 0, 0, 0, 0, 0, 'r', 'AutoScale', 'on'); % Initialize velcity vector quiver
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
zlabel('Z Position (mm)');
title('Plot of End Effector Position and Velocity');
% for i = 1:size(points, 1) - 1
%     % Calculations and movement
%     jointPos = robot.step2ik(points(i, 1:3), points(i, 4));
%     robot.set_joint_vars(jointPos, travelTimeMS);
%     pause(travelTime);
%     eeFK = robot.joints2fk(jointPos);
%     disp(size(eeFK));
%     pos = [eeFK(1,4), eeFK(2,4), eeFK(3,4)];
%     vel = robot.vel2fdk();
%     
%     % Plot ee and vv
%     set(plot3_init, 'XData', [get(plot3_init, 'XData'), pos(1)], 'YData', [get(plot3_init, 'YData'), pos(2)], 'ZData', [get(plot3_init, 'ZData'), pos(3)]);
%     set(quiver_init, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), 'UData', vel(1), 'VData', vel(2), 'WData', vel(3));
%     drawnow; % Update figure
% end
% subplot(1,3,2)
% plot(Time,Velocities)
% legend('X','Y','Z')
% title('Linear Velocity vs Time')
% xlabel('Time (S)')
% ylabel('Linear Velocity (mm/s)')