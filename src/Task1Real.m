%% Task 1 RBE 501
robot = realRobot();
robot.writeMode('curr position');
thetaList0 = [0;0;0;0];
maxTime = 10;
speedPecentage = 90;
travelTime = maxTime - (speedPecentage/100 * maxTime);
robot.writeTime(travelTime,travelTime/3)
currentA = [];
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
robot.writeMotorState(false);
disp('Torque disabled.');


