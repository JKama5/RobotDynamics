function  FollowTraj(traj,t,type)
    % robot = realRobot();
    m=Model(false);
    addpath("C:\Users\16178\Documents\WPIStuff\classes\Robotics\RBE3001_C24_Team02-main\RBE3001_C24_Team02-main\src")
    if strcmp(type,'pos')
    % robot.writeMode('curr position');
    end
    if strcmp(type,'vel')
        % robot.writeMode('velocity')
    end
    trajD=rad2deg(traj);
    resolution=t(1);
    FullTimer=tic;
    figure;
    while toc(FullTimer)<t(end)
        i=ceil(toc(FullTimer)/resolution);
        cla
        m.plot_arm(trajD(:,i)');
        drawnow
        % robot.writeJoints(traj(:,i)');
    end
end

