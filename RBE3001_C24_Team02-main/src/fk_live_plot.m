% fk_live_plot.m
function fk_live_plot()
    % Initialize the robot object from the previously defined class
    robot = Robot();
    model = Model(false);
    
    travelTime = 10; % how long to move to each pose in seconds

    %% setup Robot
    robot.writeTime(travelTime); % Write travel time
    robot.writeMotorState(true); % Write position mode
    robot.set_joint_vars([0 0 0 0],5000);%send robot home
    pause(travelTime);%wait for travel time
    disp('Home')%comunicate
    fkMatricies = zeros(4,4,5);

    % Define 5 arbitrary sets of joint angles within the robot's safe range
    setpoints = [
        10  -20 30  40
       -30  45  -60 90
        45  -45  45  -45
        30  60  -30 -60
        -45, 30, 60, -70
    ];
    
    %make figure
    figure;
    
    % Loop through each set of joint angles
    for i = 1:size(setpoints, 1)
        % Simulate moving to the setpoint over the trajectory time
        robot.set_joint_vars(setpoints(i,:));%Move
        disp('moving to position: ')
        disp(i);
        fkMatricies(:,:,i)=model.plot_arm([setpoints(i,:)])
        tic%start timer
        while toc < travelTime %while still moving 
            JointVar=robot.read_joint_vars(true,false);%get joint position [2x4] [X,Y,Z,1;X',Y',Z',1]
            jointAngles = JointVar(1,:);%get joint angles
            
            % Plot the arm using the current joint angles
            clf;%clear plot before adding a new plot
            model.plot_arm(jointAngles);
            
            pause(0.1); %pause to give time to plot
        end
    end
end
