function  FollowTraj(traj,t,type,robot) 
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
%     while toc(FullTimer)<t(end)
%         i=ceil(toc(FullTimer)/resolution);
%         if strcmp(type,'plot')
%             cla
%             m.plot_arm(trajD(:,i)');
%             drawnow
%         else
%             robot.writeJoints(trajD(:,i)');
%              pause(0.001);
%         end
%     end
    
        if strcmp(type,'plot')
            addpath("C:\Users\16178\Documents\WPIStuff\classes\Robotics\RBE3001_C24_Team02-main\RBE3001_C24_Team02-main\src")
            m=Model(false);
            while toc(FullTimer)<t(end)
                i=ceil(toc(FullTimer)/resolution);
                cla
                m.plot_arm(trajD(:,i)');
                drawnow
            end
        else
            for i=1:1:size(t,2)
                robot.writeJoints(trajD(:,i)');
                pause(resolution);
            end
        end
        
    end
end

