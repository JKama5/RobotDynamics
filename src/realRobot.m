classdef RealRobot
    % Combines kinematics with dynamic control using a Robot instance
    
    properties
        % Kinematic properties
        mlist = zeros(4, 4, 5);  % Transformation matrices for each joint
        slist;                   % Screw axis list
        glist = zeros(6, 6, 4);  % Inertial parameters
        M;                       % Home configuration

        % Reference to a Robot instance for control
        RobotInstance;
    end
    
    methods
        % Constructor
        function self = RealRobot(robot)
            % Initialize with a Robot instance
            if nargin > 0
                self.RobotInstance = robot;
            else
                error('A Robot instance must be provided.');
            end
            
            % Initialize kinematic properties
            z = 0.12800 + 0.05825 + 0.03808;
            self.slist = [0 0 0 0;
                          0 1 1 1;
                          1 0 0 0;
                          0 -(0.05825 + 0.03808) -z -z;
                          0 0 0 0;
                          0 0 0.02400 0.14800];
            
            % Set home configuration
            self.M = [0, 0, -1, 0.29345;
                      0, 1, 0, 0;
                      1, 0, 0, z;
                      0, 0, 0, 1];

            % Initialize mlist
            I = eye(3);
            mp = zeros(3, 1, 4);
            mp(:, :, 1) = [0; 0.00057; 0.07545];
            mp(:, :, 2) = [0.00486; -0.00025; 0.10353 + 0.05835 + 0.03808];
            self.mlist(:, :, 1) = [I mp(:, :, 1); 0 0 0 1];
            self.mlist(:, :, 2) = [I (mp(:, :, 2) - mp(:, :, 1)); 0 0 0 1];
            m03 = [I [0.09370 + 0.02400; -0.00001; 0.00064 + z;]; 0 0 0 1];
            self.mlist(:, :, 3) = [I (m03(1:3, 4) - mp(:, :, 2)); 0 0 0 1];
            m04 = [I [0.06424 + 0.12400 + 0.02400; -0.00001; 0.00549 + z]; 0 0 0 1];
            self.mlist(:, :, 4) = [I (m04(1:3, 4) - m03(1:3, 4)); 0 0 0 1];
            self.mlist(:, :, 5) = [self.M(1:3, 1:3) (self.M(1:3, 4) - m04(1:3, 4)); 0 0 0 1];

            % Test mlist
            TestHome = eye(4);
            for n = 1:size(self.mlist, 3)
                TestHome = TestHome * self.mlist(:, :, n);
            end
            diff = TestHome - self.M;
            if norm(diff) < 10^-7
                disp("Valid mlist");
            end
        end
        
        % Delegate control methods to Robot instance
        function writeMode(self, mode)
            self.RobotInstance.writeMode(mode);
        end

        function writeJoints(self, goals)
            self.RobotInstance.writeJoints(goals);
        end

        function readings = getJointsReadings(self)
            readings = self.RobotInstance.getJointsReadings();
        end

        function writeMotorState(self, enable)
            self.RobotInstance.writeMotorState(enable);
        end

        function writeTime(self, time, acc_time)
            self.RobotInstance.writeTime(time, acc_time);
        end

        % Add other kinematics or dynamics-related methods as needed
    end
end
