robot = realRobot(); 

task1 = Task1Functions(robot);

% Start Transformation 
startPos = robot.M;

% End Transformation 
endPos = [1 0 0 0.185;     
          0 1 0 -0.185;
          0 0 1 0.185;
          0 0 0 1];

% Set testing parameters
maxVel = 0.5;          % 50% of maximum velocity
Tf = 10;               % 10 seconds for the trajectory
initialAngles = [0; 1.5; 0; 0]; % Initial guess for joint angles

disp('Starting position control...');
try
    % Call posControl with correct name-value pair syntax
    robot.posControl(startT, endPos, maxVel, Tf, initialAngles);
    disp('Position control test completed successfully.');
catch ME
    disp('An error occurred during position control:');
    disp(ME.message);
end

