%% config
travelTime = 7;
travelTimeMS = travelTime*1000;

point1 = [80 -75 40 300]; % [x y z alpha]
point2 = [190, 0, 290, 350];
point3 = [100 65 190 340];

robot = Robot(); % create robot instance
TP= Traj_Planner(); % create trajectory planner instance
 M=Model(false);

robot.writeMotorState(true); % Write position mode
robot.set_joint_vars([0 0 0 0],3000); % send to home position
pause(3);

joints1 = robot.step2ik(point1(1:3),point1(4)); % in joint space
joints2 = robot.step2ik(point2(1:3),point2(4));
joints3 = robot.step2ik(point3(1:3),point3(4));
joints = [joints1; joints2; joints3; joints1];

T = Traj_Planner(); % instance of Traj_Planner() class

% Run
robot.set_joint_vars(joints1,travelTimeMS);

coefficents=zeros(4,4);
data=[];
for joint=1:4
    coefficents(:,joint)=TP.cubic_traj(0,travelTime,0,0,0,joints(1,joint));
end

data(:,:,1)=robot.run_trajectory(coefficents,travelTime);
for i=2:size(joints,1)
    for joint=1:4
        coefficents(:,joint)=TP.cubic_traj(0,travelTime,0,0,joints(i-1,joint),joints(i,joint));
        %TP.plotCubicTrajWithCoefficents(coefficents(:,joint),0,travelTime,1)
    end
    temp=robot.run_trajectory(coefficents,travelTime);
    if (size(data,2)>size(temp,2))
        data=data(:,1:size(temp,2));
    elseif (size(data,2)<size(temp,2))
        temp=temp(:,1:size(data,2));
    end
    data(:,:,i)=temp;
end
Pos=[];
figure;
%  M.plot_arm(robot.read_joint_vars(true,false,[true]))
Time=[data(1,:,1) travelTime+data(1,:,2) 2*travelTime+data(1,:,3) 3*travelTime+data(1,:,4)];
AllData=[data(2:end,:,1) data(2:end,:,2) data(2:end,:,3) data(2:end,:,4)];
Angles=AllData(1:4,:);
for i=1:size(Angles,2)
    Pos(i,:)=(M.fk(Angles(1:4,i)));
end
Velocities=AllData(5:8,:);
figure;
subplot(2,3,1);
plot(Time,Angles);
legend('θ1','θ2','θ3','θ4')
title('Position vs Time')
xlabel('Time (S)')
ylabel('Position (degrees)')

subplot(2,3,2)
plot(Time,Velocities)
legend('ω1','ω2','ω3','ω4')
title('Velocity vs Time')
xlabel('Time (S)')
ylabel('Angular Velocity (degrees/s)')

subplot(2,3,6)
plot(Time,AllData(9:12,:))

subplot(2,3,4)
plot(Time,AllData(13:16,:))

subplot(2,3,5)
plot(Time,AllData(17:20,:))

d2ydt2=diff(Velocities');
dt=diff(Time');
for (i=1:size(d2ydt2,1))
    d2ydt2(i,:)=d2ydt2(i,:)./dt(i);
end

subplot(2,3,3)
plot(Time(2:end),d2ydt2)
legend('θ1"','θ2"','θ3"','θ4"')
title('Angular Acceleration vs Time')
xlabel('Time (S)')
ylabel('Angular Velocity (degrees/s^2)')

figure;
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