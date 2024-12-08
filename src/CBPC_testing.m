robot = Robot();
robot.writeMode('curr position');

desired_positions = [0, 0, 0, 0];
% 
 robot.writeJoints(desired_positions);
robot.writeGripper(true)


% Example joint angles to test
% testAngles = [-45.0000
%   -10.8933
%    30.5373
%   -19.6439]; % In degrees
% 
% robot.writeJoints(testAngles);


tic;
while toc < 15
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