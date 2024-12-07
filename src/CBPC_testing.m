robot = Robot();
robot.writeMode('curr position');

desired_positions = [0, 0, -30, -70];

robot.writeJoints(desired_positions);

tic;
while toc < 15
    try
        readings = robot.getJointsReadings(); 
        present_currents_mA = readings(3, :); 
        disp(['Present currents: ', mat2str(present_currents_mA), ' mA']);
    catch ME
        warning('Unable to read present currents. Error: %s', ME.message);
    end
    
    pause(0.25); 
end

robot.writeMotorState(false);
disp('Torque disabled.');