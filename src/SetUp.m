% Get the current script's folder (workspace folder)
currentFolder = fileparts(mfilename('fullpath'));

% Move up one level to the project root folder
projectFolder = fileparts(currentFolder);

% Build the path to the library folder
ModernRoboticsFolder = fullfile(projectFolder, 'mr');
RBE3001Folder = fullfile(projectFolder,'3001');

% Add the library folder to the MATLAB path
addpath(ModernRoboticsFolder);

% (Optional) Confirm the path was added
disp(['Modern Robotics library folder added to path: ', ModernRoboticsFolder]);


%added 3001 stuff
addpath(RBE3001Folder);
disp(['RBE3001 plotting library folder added to path: ', RBE3001Folder]);


