classdef Task1Functions
    properties
        MathRobot
        M
        slist
        mlist
        glist
    end

    methods
        %constructor
        %robot: an OurRobot object
        function self=Task1Functions(robot)
            self.MathRobot=robot;
            self.M=self.MathRobot.M;
            self.slist=self.MathRobot.slist;
            self.mlist=self.MathRobot.mlist;
            self.glist=self.MathRobot.glist;
        end
        
        %find the transformation from the start to the target position
        %start: 4x4 Transformation from origin to start
        %target: 4x4 Transformation from origin to target
        %T: 4x4 transformationn from EE to Target
        function T=FindTransformations(start,target)
            T=zeros(4,4);
        end

        %find the thetalist that will get the robot to the target
        %target:4x4 Transformation from origin to target
        %thetaList: 4x1 joint angles
        function [thetaList, success] = FindThetaList(target, thetalist0)
         
            B = self.slist;
            Home = self.M;
            eomg = 1e-3;
            ev = 1e-3;
            [thetaList, success] = IKinBody(B, Home, target, thetaList0, eomg, ev);
            
            if ~success
                error('Inverse kinematics did not converge to a solution.');
            end
        end

        %Move from startPos to endPos at maxVel speed using the movment
        %functions from robot
        %startPos: 4x4 Transformation from origin to end effector
        %endPos: 4x4 Transformation from origin to target position
        %maxVel: decimal percentage of maximum velocity
        %robot: robot class that writes to the robot
        function posControl(self, startPos, endPos, maxVel, robot, Tf, thetaList0)

            % Trajectory generation
            N = 100; % Number of trajectory points
            traj = CartesianTrajectory(startPos, endPos, Tf, N, 5); % Generate trajectory
        
            % Loop through the trajectory
            for i = 1:N
                % Get target transformation at this trajectory point
                T_target = traj(:, :, i);
        
                % Solve inverse kinematics for target joint angles
                [thetaList, success] = self.FindThetaList(T_target, thetaList0);
                if ~success
                    error('Inverse kinematics failed at trajectory point %d.', i);
                end
        
                % Update thetaList0 for the next iteration
                thetaList0 = thetaList;
        
                % Write joint angles to the robot
                robot.writeJoints(rad2deg(thetaList0)); % Convert radians to degrees
        
                % Adjust speed based on maxVel
                pause(Tf / N * (1 - maxVel)); % Pause for desired velocity scaling
            end
        
            disp('Position control completed successfully.');
        end



    end
end


