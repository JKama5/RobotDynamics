%% Set up Robot
robot = realRobot();
robot.writeMode('curr position');
thetaList0 = [0; 0; 0; 0];
maxTime = 10;
speedPercentage = 50;
travelTime = maxTime - (speedPercentage/100 * maxTime);
robot.writeTime(travelTime, travelTime/3);
currentA = [];
Joint = 2;
g = [0; 0*9.81; 1*-9.81];
%% Go to Position A
A_target = [0.185; -0.185; 0.185];
A_transform = robot.FindTFromPosAndAngle(A_target);
A_orient = A_transform(1:3, 1:3);
thetaA = robot.computeJointAngles(A_target, A_orient, thetaList0);
robot.writeJoints(rad2deg(thetaA));
pause(maxTime);
%% Get Motor Currents and Calculate Theoretical Torques
disp("Reading Motor Currents...");
try
    readings = robot.getJointsReadings(); % 1x4
    currents = readings(3, :); % 1x4
    for i = 1:40
        pause(0.05);
        readings = robot.getJointsReadings(); % 1x4
        currents = [currents; readings(3, :)]; % nx4
    end
catch ME
    warning('Unable to read present currents. Error: %s', getReport(ME));
end
thetalist = deg2rad(readings(1, :)); % 1x4
thetalist = reshape(thetalist, [4, 1]); % 4x1

taulist = InverseDynamics(thetalist, [0; 0; 0; 0], [0; 0; 0; 0], ...
    g, [0; 0; 0; 0; 0; 0], robot.mlist, robot.glist, ...
    robot.slist) % 4x1
currents= mean(currents); % 1x4

%% Make an equation for torque based on current
% for i = 1:4
%     if(currents(i) == 0)
%         currents(i) = 0.00001;
%     end
% end
currents = reshape(currents, [4, 1]); % 4x1
constants = [];
for i = 1:4
    taulist(i)
    currents(i)
    if(abs(taulist(i)) > 0.001 && abs(currents(i)) > 0.001)
        constants = [constants; taulist(i) ./ currents(i)];
    end
end
% constants = taulist ./ currents; % 4x1 .* 4x1 = 4x1
constant = mean(constants, "all"); % 1x1
disp("Constant: ");
disp(constant); % 1x1
disp("Taulist: ");
disp(taulist); % 4x1
disp("Current-based estimate: ");
disp(currents .* constant); % 4x1
error = -100 * (taulist - currents .* constant)./taulist;
disp("Error: ");
disp(abs(error .* taulist)/100); % 4x1
for i = 1:4
    if(abs(error(i)) > 1e2)
        error(i) = 1e2 * error(i) / abs(error(i));
    end
end
disp("Percent Error: ");
disp((error));
%% Wait for a Wrench, then mark down current
disp("Please apply a wrench to the end effector.");
disp("Reading in 5 seconds.");
pause(1);
disp("Reading in 4 seconds.");
pause(1);
disp("Reading in 3 seconds.");
pause(1);
disp("Reading in 2 seconds.");
pause(1);
disp("Reading in 1 second.");
pause(1);
disp("Reading...");
try
    readings = robot.getJointsReadings(); % 1x4
    currents = readings(3, :); % 1x4
    for i = 1:40
        pause(0.05);
        readings = robot.getJointsReadings(); % 1x4
        currents = [currents; readings(3, :)]; % nx4
    end
    currents = mean(currents); % 1x4
    currents = reshape(currents, [4, 1]); % 4x1
    disp("Currents: ");
    disp(currents); % 4x1
catch ME
    warning('Unable to read present currents. Error: %s', getReport(ME));
end
torques = currents .* constant; % 4x1 .* 1x1 = 4x1
%% Calculate Torque, compare to theoretical torque
Ftip = [0; 0; 0; -0*9.81; -0*9.81; 0*9.81];
taulist = InverseDynamics(thetalist, [0; 0; 0; 0], [0; 0; 0; 0], g, Ftip, robot.mlist, robot.glist, robot.slist);
disp("Theoretical Torques: ");
disp(taulist); % 4x1
disp("Calculated Torques: ");
disp(torques); % 4x1
disp("Error: ");
disp(taulist - torques); % 4x1
error = -100 * (taulist - torques)./taulist;
for i = 1:4
    if(abs(error(i)) > 1e2)
        error(i) = 1e2 * error(i) / abs(error(i));
    end
end
disp("Percent Error: ");
disp(error);