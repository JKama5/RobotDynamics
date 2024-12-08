%% Task 1 RBE 501
robot = realRobot();
robot.writeMode('curr position');
thetaList0 = [0;0;0;0];
maxTime = 10;
speedPecentage = 60;
travelTime = maxTime - (speedPecentage/100 * maxTime);
robot.writeTime(travelTime,travelTime/3)
currentA = [];
currentB = [];
Joint = 2;
%%%

A_target = [0.185; -0.185; 0.185];
B_target = [0.185; 0.170; 0.07];

A_transform = robot.FindTFromPosAndAngle(A_target) ;
B_transform = robot.FindTFromPosAndAngle(B_target);

A_orient = A_transform(1:3,1:3);
B_orient = B_transform(1:3,1:3);

thetaA = robot.computeJointAngles(A_target, A_orient,thetaList0);
thetaB = robot.computeJointAngles(B_target, B_orient,thetaList0);

robot.writeJoints(rad2deg(thetaA));

tic;
while toc < travelTime
    try
        readings = robot.getJointsReadings();
        present_currents_mA = readings(3, Joint); 
        currentA = [currentA,present_currents_mA];
    catch ME
        warning('Unable to read present currents. Error: %s', ME.message);
    end
    pause(0.01); 
end
% robot.writeMotorState(false);
% disp('Torque disabled.');

if speedPecentage == 80
    current_80_A = currentA;
end
if speedPecentage == 60
    current_60_A = currentA;
end
if speedPecentage == 40
    current_40_A = currentA;
end
 
robot.writeTime(travelTime,travelTime/3)

robot.writeJoints(rad2deg(thetaB));

tic;
while toc < travelTime
    try
        readings = robot.getJointsReadings();
        present_currents_mA = readings(3, Joint); 
        currentB = [currentB,present_currents_mA];
    catch ME
        warning('Unable to read present currents. Error: %s', ME.message);
    end
    pause(0.01); 
end

if speedPecentage == 80
    current_80_B = currentB;
end
if speedPecentage == 60
    current_60_B = currentB;
end
if speedPecentage == 40
    current_40_B = currentB;
end

t1 = linspace(0, 6, 126); % 125 points from 0 to 6
% Adjust X-axis for each array based on their lengths
t2 = linspace(0, 4, 84);
t3 = linspace(0, 2, 42);
% Create a figure and hold it for multiple plots
figure;
hold on;
plot(t1, current_40_B, '-o', 'DisplayName', '40% Speed', 'LineWidth', 1.5);
plot(t2, current_60_B, '-s', 'DisplayName', '60% Speed', 'LineWidth', 1.5);
plot(t3, current_80_B, '-^', 'DisplayName', '80% Speed', 'LineWidth', 1.5);

% Add labels, title, and legend
xlabel('Time (s)');
ylabel('Current [mA]');
title('Current Over Time for Joint 2');
legend('show', 'FontSize', 14);
ax = gca; % Get current axes
ax.FontSize = 12;
% Turn off hold
hold off;