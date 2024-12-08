robot = realRobot();
robot.writeMode('curr position');
thetaList0 = [0;0;0;0];
A_target = [0.185; 0; .240];

A_transform = robot.FindTFromPosAndAngle(A_target) ;

A_orient = A_transform(1:3,1:3);

thetaA = robot.computeJointAngles(A_target, A_orient,thetaList0);

robot.writeJoints(rad2deg(thetaA));

tic;
while toc < 6
    try
        readings = robot.getJointsReadings();
        present_position = readings(1,:);
        present_currents_mA = readings(3, :); 
        disp(['Present currents: ', mat2str(present_currents_mA), ' mA']);
        disp(['Present position: ', mat2str(present_position),]);

    catch ME
        warning('Unable to read present currents. Error: %s', ME.message);
    end

    pause(0.25); 
end


robot.writeMotorState(false);
disp('Torque disabled.');