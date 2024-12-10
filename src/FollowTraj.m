function  FollowTraj(traj,t,type) 
    trajD=rad2deg(traj);
    resolution=t(1);
    FullTimer=tic;
    figure;
        if strcmp(type,'plot')
            SetUp;
            m=Model(false);
            while toc(FullTimer)<t(end)
                i=ceil(toc(FullTimer)/resolution);
                cla
                m.plot_arm(trajD(:,i)');
                drawnow
            end
        else
            robot= realRobot();
            if strcmp(type,'pos')
                robot.writeMode('curr position');
            elseif strcmp(type,'vel')
                robot.writeMode('velocity') 
            end
            for i = 1:size(traj, 2)
                if strcmp(type,'pos')
                    robot.writeMode('curr position');
                    robot.writeJoints(trajD(:, i)'); % Send joint commands to the robot
                elseif strcmp(type,'vel')
                    robot.writeMode('velocity') 
                    robot.writeVelocities(trajD(:,i)');
                end
                pause(resolution); % Wait for the next time step
            end
        end
end

