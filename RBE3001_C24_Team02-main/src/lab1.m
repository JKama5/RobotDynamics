
%% Setup robot
travelTime = 6; % Defines the travel time
robot = Robot(); % Creates robot object
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

% % Arrays to store the data for all repetitions for the histogrmas
allPositions = [];
allTimestamps = [];
allTimeDiffs = [];

%% Program 

for index = 1:3 %preform 3 repetitions
    robot.writeJoints(0); % Write joints to zero position
    pause(travelTime); % Wait for trajectory completion
    
    robot.set_joint_vars([45,0,0,0]); %set the base joint to 45 degrees
    
    pos_arr = []; % array of unkown size to store pos values
    time_arr = []; % array of unknown size to store time values in ms
    i = 1; % index to inc over rows
    
    tic; % Start timer
    
    %while moving the arm
    while toc < travelTime
        pos = robot.read_joint_vars(true, false); % read joint position values
        pos_arr(i, :) = pos(1, :); % update joint position values
        time_arr(i, :) = toc*DX_XM430_W350.MS_PER_S; %store reading timestamp
        i=i+1; % increment
    end
    

    if (index==1) % if preforming the first repetition
        % plot position of all joints during first repetition
        plot(time_arr, pos_arr);
        title('All Joint Movements for 1 Repetition');
        xlabel('Time (ms)');
        ylabel('Position (degrees)');
        legend('Base', 'Shoulder', 'Elbow', 'Wrist');
        
        % Save the figure as a PNG file
        saveas(gcf, 'all_joint_movement.png');

        %Create the plot for the graph of the base position during all 3
        %repetitions and add the data from the first repetition 
        figure;% Create figure to check that data is being applied to the correct plot
        plot(time_arr,pos_arr(:,1))
        hold on; %allow next plot call to add to this graph 
    end


    if index~=1% if not preforming the first rotation
        %disp("plotting not 1")%debugging test  
        plot(time_arr,pos_arr(:,1))% add data from this repetition to the graph of the base position during all 3 repetitions
    end

    %disp(index) %debugging test
    
    allPositions = [allPositions; pos_arr]; % Store positions from this iteration
    allTimestamps = [allTimestamps; time_arr]; % Store timestamps from this iteration
    
    % Calculate time differences for this repetition
    timeDiffs = diff(time_arr); % Use newly collected timestamps
    allTimeDiffs = [allTimeDiffs; timeDiffs]; % Append to the overall time differences
end 


%Lable the graph of the base position during all 3 repetitions
hold off;% prevent further plots from effecting this graph
title('Base Joint Movements for 3 Repetitions');
xlabel('Time (ms)');
ylabel('Position (degrees)');
legend('Repetition 1', 'Repetition 2', 'Repetition 3');
saveas(gcf, 'Base_Joint_Movements_for_3_Repetitions.png'); %save plot as a png

%% %Everything bellow is the original code from when Manas first did the lab (parts were copy and pasted and used in the above code and the histogram code is still being used)        
% % Initialize the Robot
% robot = Robot();
% 
% % Define the starting and ending positions for the base joint, and travel time
% startPosition = 0; % Starting angle for the base joint
% endPosition = 45; % Ending angle for the base joint
% travelTime = 3; % Travel time in seconds
% 
% % Set up the robot to the starting position
% robot.set_joint_vars([startPosition, 0, 0, 0]);
% pause(travelTime);
% 
% % Define the number of repetitions
% repetitions = 3;
% 
% % Arrays to store the data for all movements
% allPositions = [];
% allTimestamps = [];
% allTimeDiffs = [];
% 
% % Repeat the movement and collect data
% for i = 1:repetitions
%     % Move from startPosition to endPosition
%     robot.set_joint_vars([endPosition, 0, 0, 0], travelTime * 1000);
%     pause(travelTime + 1); % Give extra time to settle
%     
%     % Collect the data while moving
%     tic;
%     positions = []; % Initialize positions array
%     timestamps = []; % Initialize timestamps array
%     while toc < travelTime
%         currentReadings = robot.getJointsReadings();
%         positions = [positions; currentReadings(1, :)]; 
%         timestamps = [timestamps; toc * 1000]; % Collect timestamps in milliseconds
%         pause(0.01); % Adjust as needed based on your setup 
%     end
%     allPositions = [allPositions; positions]; % Store positions from this iteration
%     allTimestamps = [allTimestamps; timestamps]; % Store timestamps from this iteration
%     
%     % Calculate time differences for this repetition
%     timeDiffs = diff(timestamps); % Use newly collected timestamps
%     allTimeDiffs = [allTimeDiffs; timeDiffs]; % Append to the overall time differences
%     
%     % Move back to the starting position
%     robot.set_joint_vars([startPosition, 0, 0, 0], travelTime * 1000);
%     pause(travelTime + 1); % Give extra time to settle
% end
% % ...
% 
% % Plot the 3 base joint movements on the same plot.
% figure;
% plot(allTimestamps, allPositions);
% title('Base Joint Movements for 3 Repetitions');
% xlabel('Time (ms)');
% ylabel('Position (degrees)');
% legend('Repetition 1', 'Repetition 2', 'Repetition 3');
% 
% % Save the figure as a PNG file
% saveas(gcf, 'base_joint_movements.png');

% ...

% Plot the timing histogram with and without outliers.

% Flatten the time differences array for analysis
allTimeDiffsFlat = allTimeDiffs(:);

% Calculate the Interquartile Range (IQR)
Q1 = quantile(allTimeDiffsFlat, 0.25);
Q3 = quantile(allTimeDiffsFlat, 0.75);
IQR = Q3 - Q1;

% Identify outliers using the IQR method
isOutlier = (allTimeDiffsFlat < (Q1 - 1.5 * IQR)) | (allTimeDiffsFlat > (Q3 + 1.5 * IQR));

% Data without outliers
dataWithoutOutliers = allTimeDiffsFlat(~isOutlier);

% Plot histogram with outliers
figure;
histogram(allTimeDiffsFlat);
title('Timing Histogram with Outliers');
xlabel('Time Difference (ms)');
ylabel('Frequency');

% Save the histogram with outliers as a PNG file
saveas(gcf, 'timing_histogram_with_outliers.png');

% Plot histogram without outliers
figure;
histogram(dataWithoutOutliers);
title('Timing Histogram without Outliers');
xlabel('Time Difference (ms)');
ylabel('Frequency');

% Save the histogram without outliers as a PNG file
saveas(gcf, 'timing_histogram_without_outliers.png');
%}


