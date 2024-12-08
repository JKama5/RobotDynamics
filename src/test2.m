robot = realRobot();
robot.writeMode('curr position');
angle = [0;0;0;0];
maxTime = 10;
speedPecentage = 90;
travelTime = maxTime - (speedPecentage/100 * maxTime);
robot.writeTime(travelTime,travelTime/3)
robot.writeJoints(angle)
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