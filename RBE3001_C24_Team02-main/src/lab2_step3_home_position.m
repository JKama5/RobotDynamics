%% Setup robot
travelTimeMS = 3000; % Defines the travel time
travelTimeS = travelTimeMS/1000; % use in pause
robot = Robot(); % Creates robot object
%robot.writeTime(travelTime/1000); % Write travel time
robot.writeMotorState(true); % Write position mode

%% Begin program
targetJoints = [-45,30,60,-70];

robot.set_joint_vars([0,0,0,0],travelTimeMS); % Write joints to zero position
pause(travelTimeS); % Wait for trajectory completion

bigMat = zeros(4,24); % expected matrix size
sumCoords = zeros(1,3); % expected sum matrix size
%=========================================================================%
for index = 1:5 % perform 5 repetitions
    robot.set_joint_vars(targetJoints,travelTimeMS); % set the joints 
    pause(travelTimeS); % Wait for trajectory completion

    readJoints = robot.read_joint_vars(true, false); % read joint position values
    jointsDeg(index,:) = readJoints(1,:); % store in matrix

    robot.set_joint_vars([0,0,0,0],travelTimeMS); % Write joints to zero position
    pause(travelTimeS); % Wait for trajectory completion

    indexMat = index+(index-1)*4; % index to format bigMat correctly
    fk_mat = robot.joints2fk(readJoints(1,:)); % returns 4x4xn matrix
    bigMat(:,indexMat:indexMat+3) = robot.BaseToTipT(fk_mat); % big matrix containing 5 4x4 homogeneous transformations
end
%=========================================================================%
%% Calculate expected value
expectedStore = robot.BaseToTipT(robot.joints2fk(targetJoints)); % stores to variable
expected = expectedStore(1:3,4); % saves translation

%% Plot data
plot3(0,0,0,'.','Color','k'); % origin

hold on
for index = 1:5
    indexMat = index+(index-1)*4; % index to navigate bigMat correctly
    X = bigMat(1,indexMat+3); 
    Y = bigMat(2,indexMat+3);
    Z = bigMat(3,indexMat+3);

    rms(expected - [X Y Z]) % print rms of error

    sumCoords = [sumCoords(1)+X, sumCoords(2)+Y, sumCoords(3)+Z]; % take sum over time
    plot3(X,Y,Z,'.');
end

average = sumCoords/5; % calculate average x,y,z
plot3(average(1),average(2),average(3),'.','Color','r');
plot3(expected(1),expected(2),expected(3),'.','Color','g');

%% Graph stuff
legend({'Origin','Point 1','Point 2','Point 3','Point 4','Point 5','Average','Expected'},"AutoUpdate","off");

grid on; % turn on the grid
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

xlim([-400 400]);
ylim([-400 400]);
zlim([0 400]);

title('Base to Tip Transformation of End Effector');
% quiver3(0,0,0,10,0,0,'g','AutoScale','off'); % world frame x
% quiver3(0,0,0,0,10,0,'m','AutoScale','off'); % world frame y 
% quiver3(0,0,0,0,0,10,'b','AutoScale','off'); % world frame z
hold off
