%% Setup robot
pos = zeros(4,60*3);%initialize robot position [X;Y;Z;1]
angles = zeros(4,60*3);
travelTime = 6; % Defines the travel time


robot = Robot(); % Creates robot object
model = Model(true);%Creates a model object with the robot attached
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
robot.set_joint_vars([0 0 0 0]);%send robot home
pause(travelTime);%wait for travel time
disp('Home')%comunicate
index=1;%store which value of pos we are at


%% move to pose 1
robot.set_joint_vars([0 20 0 0]);%Move
pause(travelTime);
disp('@ pos 1')%comunicate

%% Move to pose 2
robot.set_joint_vars([0 -20 60 20]);%Move

%while moving record the tip position and all the angles
    tic; % Start timer
    while toc < travelTime
        pos(:, index) = model.ee_pos();
        thing=robot.read_joint_vars(true,false);
        angles(:,index)=transpose(thing(1,:));
        index=index+1;
        pause(0.1); %take a reading every 10th of a second
    end

disp('@ pose 2')%comunicate

%% Move to pose 3
robot.set_joint_vars([0 -30 10 10]);%Move
    
%while moving record the tip position
    tic; % Start timer
    while toc < travelTime
        pos(:, index) = model.ee_pos();
        thing=robot.read_joint_vars(true,false);
        angles(:,index)=transpose(thing(1,:));
        index=index+1;
        pause(0.1); %take a reading every 10th of a second
    end

disp('@ pose 3')%comunicate

%% Return to pose 1
robot.set_joint_vars([0 20 0 0]);%Move

%while moving record the tip position
    tic; % Start timer
    while toc < travelTime
        pos(:, index) = model.ee_pos();
        thing=robot.read_joint_vars(true,false);
        angles(:,index)=transpose(thing(1,:));
        index=index+1;
        pause(0.1); %take a reading every 10th of a second
    end

disp('pos 1 again')%comunicate

%Remove all the 0 columns at the end of pos
zeroColumns= all(pos==0);
pos(:,zeroColumns)=[];
zeroColumns=all(angles==0)
angles(:,zeroColumns)=[];

figure;%make a blank figure
plot3(pos(1,:),pos(2,:),pos(3,:))%plot the position from pos

%set axis limits
xlim([-400,400]);
ylim([-400,400]);
zlim([0,400]);

output=zeros(7,size(pos,2));
output(1:3,:)=pos(1:3,:);
output(4:7,:)=angles;
csvwrite('Lab2Step6TrianglePlotData.csv',output);%save pos to a csv

% function recordEEPos()
%     global travelTime
%     global pos
%     global model
%     tic; % Start timer
%     while toc < travelTime*1000
%         pos(:, end+1) = model.ee_pos();
%         pause(0.1); % Adjust the pause time if needed
%     end
% end
