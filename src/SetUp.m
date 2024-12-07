% Get the current script's folder (workspace folder)
currentFolder = fileparts(mfilename('fullpath'));

% Move up one level to the project root folder
projectFolder = fileparts(currentFolder);

% Build the path to the library folder
libraryFolder = fullfile(projectFolder, 'mr');

% Add the library folder to the MATLAB path
addpath(libraryFolder);

% (Optional) Confirm the path was added
disp(['Library folder added to path: ', libraryFolder]);