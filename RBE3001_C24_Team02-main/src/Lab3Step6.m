%% config
travelTime = 22;
travelTimeMS = travelTime*1000;

point1 = [80 -75 40 300]; % [x y z alpha]
point2 = [190, 0, 290, 350];
point3 = [100 65 190 340];
points = [point1; point2; point3; point1];
pointsOrigin = [281.4 0 224.3254 360]; % [x y z alpha] origin

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

%T = Traj_Planner(); % instance of Traj_Planner() class

%% Run
%robot.set_joint_vars(joints1,travelTimeMS);
coefficents=zeros(4,4);
data=[];
for point=1:4
    coefficents(:,point)=TP.cubic_traj(0,travelTime,0,0,pointsOrigin(1,point),points(1,point));
end

data(:,:,1)=robot.run_trajectory(coefficents,travelTime,true);

% robot.read_joint_vars(true,true)
for i=2:size(points,1)
    for point=1:4
        coefficents(:,point)=TP.cubic_traj(0,travelTime,0,0,points(i-1,point),points(i,point));
        %TP.plotCubicTrajWithCoefficents(coefficents(:,joint),0,travelTime,1)
    end
    temp=robot.run_trajectory(coefficents,travelTime,true); % blocking
    
    %Code to handle sizes of data collected from each side of the triangle
    % not matching 
    if (size(data,2)>size(temp,2))
        data=data(:,1:size(temp,2));
    elseif (size(data,2)<size(temp,2))
        temp=temp(:,1:size(data,2));
    end
    data(:,:,i)=temp;
end

%%Plotting
Pos=[];
Time=[data(1,:,1) travelTime+data(1,:,2) 2*travelTime+data(1,:,3) 3*travelTime+data(1,:,4)];
AllData=[data(2:end,:,1) data(2:end,:,2) data(2:end,:,3) data(2:end,:,4)];
Angles=AllData(1:4,:);
for i=1:size(Angles,2)
    Pos(i,:)=(M.fk(Angles(1:4,i)));
end
Velocities=AllData(5:8,:);
JS=figure;
set(JS, 'Name', 'Step 6 Joint Space')
subplot(1,3,1);
plot(Time,Angles);
legend('θ1','θ2','θ3','θ4')
title('Position vs Time')
xlabel('Time (S)')
ylabel('Position (degrees)')

subplot(1,3,2)
plot(Time,Velocities)
legend('ω1','ω2','ω3','ω4')
title('Velocity vs Time')
xlabel('Time (S)')
ylabel('Angular Velocity (degrees/s)')

d2ydt2=diff(Velocities');
dt=diff(Time');
for (i=1:size(d2ydt2,1))
    d2ydt2(i,:)=d2ydt2(i,:)./dt(i);
end

subplot(1,3,3)
plot(Time(2:end),d2ydt2)
legend('θ1"','θ2"','θ3"','θ4"')
title('Angular Acceleration vs Time')
xlabel('Time (S)')
ylabel('Angular Velocity (degrees/s^2)')

TS=figure;
set(TS,'Name','Step 6 Task Space')
subplot(1,3,1)
plot(Time,Pos);
legend('X','Y','Z')
title('End Effector Position vs Time')
xlabel('Time (S)')
ylabel('Position (mm)')

dpdt=diff(Pos);
for (i=1:size(Time,2)-1)
    d2ydt2(i,:)=d2ydt2(i,:)./dt(i);
end
subplot(1,3,2)
plot(Time(1:end-1),dpdt)
legend('dX/dt','dY/dt','dZ/dt')
title('End Effector Velocity vs Time')
xlabel('Time (S)')
ylabel('Velocity (mm/s)')

d2pdt2=diff(dpdt);
for (i=1:size(Time,2)-2)
    d2pdt2(i,:)=d2pdt2(i,:)./dt(i);
end
subplot(1,3,3)
plot(Time(1:end-2),d2pdt2)
legend('X"','Y"','Z"')
title('End Effector Accelerration vs Time')
xlabel('Time (S)')
ylabel('Position (mm/s^2)')

figure;
X=Pos(:,1)';
Y=Pos(:,2)';
Z=Pos(:,3)';
plot3(X,Y,Z)
title('3D Path of End Effector in Task Space')
xlabel('X Position (mm)')
ylabel('Y Position (mm)')
zlabel('Z Position (mm)')