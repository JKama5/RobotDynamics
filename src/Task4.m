%% Set up Robot
robot = realRobot();
robot.writeMode('curr position');
thetaList0 = [0; 0; 0; 0];
maxTime = 4;
speedPercentage = 90;
travelTime = maxTime - (speedPercentage/100 * maxTime);
robot.writeTime(travelTime, travelTime/3);
currentA = [];
Joint = 2;

%% Go to Position A
A_target = [0.185; -0.185; 0.185];
A_transform = robot.FindTFromPosAndAngle(A_target);
A_orient = A_transform(1:3, 1:3);
thetaA = robot.computeJointAngles(A_target, A_orient, thetaList0);
robot.writeJoints(rad2deg(thetaA));
pause(maxTime);
%% Get Motor Currents and Calculate Theoretical Torques
sleep(1);
try
    readings = robot.getJointsReadings();
    currents = readings(3, :);
catch ME
    warning('Unable to read present currents. Error: %s', getReport(ME));
end
thetalist = readings(1, :);
taulist = InverseDynamics(thetalist, [0; 0; 0; 0], [0; 0; 0; 0], [0; 0; -9.81], [0; 0; 0; 0; 0; 0; 0], robot.mlist, robot.Glist, robot.slist);
%% Make an equation for torque based on current
constant = average(taulist ./ currents);
torque = @(current) constant * current;
%% Wait for a Wrench, then mark down current
disp("Please apply a wrench to the end effector.");
pause(5);
try
    readings = robot.getJointsReadings();
    currents = readings(3, :);
    disp("Currents: ");
    disp(currents);
catch ME
    warning('Unable to read present currents. Error: %s', getReport(ME));
end
%% Calculate Torque, compare to theoretical torque
torques = torque(currents);
Ftip = [0; 0; 0; 0; 0; 0; 0];
taulist = InverseDynamics(thetalist, [0; 0; 0; 0], [0; 0; 0; 0], [0; 0; -9.81], Ftip, robot.mlist, robot.Glist, robot.slist);
disp("Theoretical Torques: ");
disp(taulist);
disp("Calculated Torques: ");
disp(torques);