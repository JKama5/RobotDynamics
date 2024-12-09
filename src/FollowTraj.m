function  FollowTraj(traj,t,type) 
    m=Model(false);
    addpath("C:\Users\16178\Documents\WPIStuff\classes\Robotics\RBE3001_C24_Team02-main\RBE3001_C24_Team02-main\src")
    if strcmp(type,'pos')
        robot = realRobot();
        robot.writeMode('curr position');
    elseif strcmp(type,'vel')
        robot = realRobot();
        robot.writeMode('velocity') 
    end
    trajD=rad2deg(traj);
    resolution=t(1);
    FullTimer=tic;
    figure;
    while toc(FullTimer)<t(end)
        i=ceil(toc(FullTimer)/resolution);
        if strcmp(type,'plot')
            cla
            m.plot_arm(trajD(:,i)');
            drawnow
        else
            robot.writeJoints(traj(:,i)');
        end
    end
end

