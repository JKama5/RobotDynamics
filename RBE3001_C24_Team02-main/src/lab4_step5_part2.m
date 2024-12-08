%% config
travelTime = 12;
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
coefficents=zeros(6,4);
data=[];
for point=1:4
    coefficents(:,point)=TP.quintic_traj(0,travelTime,0,0,0,0,pointsOrigin(1,point),points(1,point));
end

data(:,:,1)=robot.run_trajectory(coefficents,travelTime,true);

% robot.read_joint_vars(true,true)
for i=2:size(points,1)
    for point=1:4
        coefficents(:,point)=TP.quintic_traj(0,travelTime,0,0,0,0,points(i-1,point),points(i,point));
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

%% 
%%Plotting
Pos=[];
Time=[data(1,:,1) travelTime+data(1,:,2) 2*travelTime+data(1,:,3) 3*travelTime+data(1,:,4)];
AllData=[data(2:end,:,1) data(2:end,:,2) data(2:end,:,3) data(2:end,:,4)];

%% 
Angles=AllData(1:4,:);
disp("calculating position from fk");
for i=1:size(Angles,2)
    i
    disp("out of: ");
    disp(size(Angles,2))
    Pos(i,:)=(M.fk(Angles(1:4,i)));
end
%% Timeing breakdown
setupTime=mean(AllData(5,:))
CalculatePosition=mean(AllData(6,:))
CalculateTaskSpace=mean(AllData(7,:))
ReadJoints=mean(AllData(8,:))
DataHandeling=mean(AllData(9,:))
outOfLoop=mean(AllData(10,:))
testdeltaT=setupTime+CalculatePosition+CalculateTaskSpace+ReadJoints+DataHandeling+outOfLoop
deltaT=mean(diff(Time))
deltaT=deltaT/100
%DataHandeling/deltaT*100

%% 
% Velocities=AllData(5:8,:);
% 
% JS=figure;
% set(JS, 'Name', 'Step 6 Joint Space')
% 
% TaskSpaceVel=[];
% for i=1:size(AllData,2)
%     i
%     disp("out of: ");
%     disp(size(AllData,2))
%     TaskSpaceVel(:,i)=M.lab2step2.vel2fdk(Angles(:,i),Velocities(:,i));
% end
% 
% subplot(1,3,1);
% plot(Time,TaskSpaceVel(1:3,:));
% legend('Linear Velocity in X','Linear Velocity in Y','Linear Velocity in Z')
% title('Linear Velocity vs Time')
% xlabel('Time (s)')
% ylabel('Position (mm)')
%% 
dt=diff(Time);
figure;
plot(dt)
