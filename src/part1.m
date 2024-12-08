% Initialize the realRobot instance
robot = realRobot(); % Create an instance of realRobot

% Create an instance of Task1Functions, passing the robot object
task1 = Task1Functions(robot);

% Define start and end transformations
% Start Transformation (4x4 homogeneous matrix)
startPos = robot.M;

% End Transformation (4x4 homogeneous matrix)
endPos = [1 0 0 0.185;     % Example: Replace with your actual end position
          0 1 0 -0.185;
          0 0 1 0.185;
          0 0 0 1];

% Set testing parameters
maxVel = 0.5;          % 50% of maximum velocity
Tf = 10;               % 10 seconds for the trajectory
initialAngles = [0; 1.5; 0; 0]; % Initial guess for joint angles

% Call posControl to move the robot
disp('Starting position control...');
try
    % Call posControl with correct name-value pair syntax
    robot.posControl(startT, endPos, maxVel, Tf, initialAngles);
    disp('Position control test completed successfully.');
catch ME
    disp('An error occurred during position control:');
    disp(ME.message);
end

