% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001
classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        defaultTravelTime = 5000;
        lab2Step2; %stores the instance of the Lab2Step2 class that contains the functions from step 2 of lab 2
        model;
    end
    
    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            %
            self.lab2Step2=Lab2Step2();
            self.model=Model(false);
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);
            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);
            % Robot Dimensions
            self.mDim = [96.326, 130.23, 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
        end
        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
            self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
        end
        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
      
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end
            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;
%             disp("time")
%             disp(time_ms)
%             disp("acc time")
%             disp(acc_time_ms)
            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end
        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end
        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end
        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end
            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);
            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end
        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XMdefaultTravelTime430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end
        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end
        
        % 1x4 array of joint variables (in degrees) to be sent to the actuators
        % optional: desired traveltime (ms)
        function set_joint_vars(self, jointVars, varargin)
            if ~isempty(varargin)
                travelTime = varargin{1};
            else
                travelTime = self.defaultTravelTime;
            end
            travelTimeInSeconds = travelTime / DX_XM430_W350.MS_PER_S; % ms / 1000 s
            self.writeTime(travelTimeInSeconds);
            self.writeJoints(jointVars);
        end
        % input: self instance of robot
        % input: GETPOS boolean if true returns position array in degrees
        % input: GETVEL boolean if true returns velocity array
        % input: varargin boolean if true returns as 1 row
        % returns: [2 4] array, first row degree, second velocity
        %                       if bool false, set to 0
        function [pos_vel_arr] = read_joint_vars(self, GETPOS, GETVEL, varargin)
            posRow=1;
            VelRow=2;
            if isempty(varargin)
                pos_vel_arr = zeros(2,4);
            elseif varargin{1}==true
                pos_vel_arr = zeros(1,4);
                VelRow=1;
            end
            if GETPOS
                pos_vel_arr(posRow, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;                            
            end 
            if GETVEL
                pos_vel_arr(VelRow, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
                pos_vel_arr(VelRow,2)=pos_vel_arr(VelRow,2)-61.83;
                pos_vel_arr(VelRow,3)=pos_vel_arr(VelRow,3)-65.9520;
            end
        end 

        function [pos_vel_arr] = read_joint_vars2(self, GETPOS)
            if GETPOS
                pos_vel_arr(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;                            
            else
                pos_vel_arr(1, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
                pos_vel_arr(1,2)=pos_vel_arr(VelRow,2)-61.83;
                pos_vel_arr(1,3)=pos_vel_arr(VelRow,3)-65.9520;
            end
        end 
        
        % input dhArr [1x4] array corresponding to a row of the DH parameter table for a given link 
        %                   [θ d a α]
        % return transform [4x4] homogeneous transformation matrix 
        function dhTransform = dh2mat(self,dhArr)
            dhTransform=self.lab2Step2.dh2mat(dhArr);%uses the function in the lab 2 step 2 class
        end
        
        % input dhMat [nx4] array corresponding to the n rows of the full DH parameter table, 
        %                   rows take form [θ d a α]
        % return compTransform [4x4]xn array where "n" is the joint number, and each joint gets 
        %                              a 4x4 homogenous matrix that gives the global orientation and position 
        function compTransform = dh2fk(self,dhMat)
            compTransform=self.lab2Step2.dh2fk(dhMat);%uses the function in the lab 2 step 2 class
        end
        
        % input jointVar [1x4] array of the joint variables in degrees
        % return compTransform [4x4]xn array where "n" is the joint number, and each joint gets 
        %              a 4x4 homogenous matrix that gives the global orientation and position 
        function compTransform= joints2fk(self, jointVarDeg)
            compTransform=self.lab2Step2.joints2fk(jointVarDeg);%uses the function in the lab 2 step 2 class
        end
        
        %input: [4x4]xn array output by dh2fk (array of all the individual
        %   transformation matrices
        %output: [4x4] array of the resulting transformation matrix
        function T = BaseToTipT(self, in)
            T=self.lab2Step2.BaseToTipT(in);%uses the function in the lab 2 step 2 class
        end
        % perform inverse kinimatics to get from desired position and
        %   orientation to joint angles =
        % p: 1x3 vector of desired position: [x,y,z]
        % a: angle the end effector makes with the x axis mesured from a
        %   line parrallel to the x axis going through joint 4 to the
        %   underside of the end effector
        %q: 1x4 vector of the 4 joint angles: q1, q2, q3, q4
        %           joint limits: q1:[-90,90] q2: [-90. 90] q3: [-90,80]
        %           q4:[-90, 90]
        function q = step2ik(self,p,a)
            q = self.lab2Step2.step2ik2(p,a);
        end

        %Has the robot follow the trajectory generated by the coefficents
        %   passed into the fucntion 
        %trajCoeffcients: This will be a 4x4 matrix or 6x4 (4 OR 6 coefficients, 4 joints OR [x y z alpha]) of trajectory coefficients which are generated from
        %   cubic_traj() or quintic_traj(), respectively.
        %time: The total amount of time the trajectory should take. This must be the same duration you passed to cubic_traj() to generate the trajectories for each joint.
        %spaceBool: specifies whether trajectory is joint space or task
        %           space (column data) (TRUE = task, FALSE = joint)
        %varargin: optional final position used to recalibrate if
        %          singularity detected, only works for cubic and task
        %          space
        %data: for plotting
        function data=run_trajectory(self,trajCoefficients,time,spaceBool)
            Timer = tic;
            a=trajCoefficients;
            InitialRealTimes=zeros(1,20);
            InitialJointPos=zeros(4,20);
            Timers=zeros(6,5000);
            tindex=1;
            testRangeIndex=size(InitialRealTimes,2);
            innerTimer=tic;
            %t=0;
            %deltat=0.03;
            Test=tic;
            while toc(Timer)<time
%             thingymebober=1;
%             while thingymebober<time
                outOfLoopT=toc(Test);
                Test=tic;
                t=toc(Timer);
%                 t=thingymebober
                %t=t+deltat;
                p = zeros(4); % allocate mem
                setupT=toc(Test);
                %=========================================================%
                if(size(a,1)==4) % check if cubic_traj
                    for index=1:4 % for each col of trajCoefficients which can be joint or task space (theta1,theta2,thea3,theta4 OR x,y,z,alpha)
                        p(index)=a(1,index)+a(2,index)*t+a(3,index)*t^2+a(4,index)*t^3; % returns a 1x4 position array
                    end
                else % quintic_traj, so 6 rows
                    for index=1:4 % for each col of trajCoefficients (theta1,theta2,thea3,theta4 OR x,y,z,alpha)
                        p(index)=a(1,index)+a(2,index)*t+a(3,index)*t^2+a(4,index)*t^3+a(5,index)*t^4+a(6,index)*t^5; % returns a 1x4 position array
                    end
                end
                CalculatePositionT=toc(Test)-setupT;
                if(spaceBool) % if in task space, find ik 
                    p = self.step2ik(p(1:3),p(4));
                end
                CalculateTaskSpaceT=toc(Test)-CalculatePositionT-setupT;
                %=========================================================%
                
                % for plotting
                
                RealVars=self.read_joint_vars2(true)';
                ReadJointsT=toc(Test)-CalculateTaskSpaceT-CalculatePositionT-setupT;
                
                if (tindex<testRangeIndex)
                    deltat=toc(innerTimer); % time it takes for code to get to this point on every loop
                    self.set_joint_vars(p,200); % control motors to go to joint degrees
                    innerTimer=tic; % start timer for next code loop
                    InitialRealTimes(1,tindex)=t;
                    InitialJointPos(:,tindex)=RealVars(:,1);
                elseif tindex==testRangeIndex
                    deltat=mean(abs(diff(InitialRealTimes)));
                    self.set_joint_vars(p,200); % control motors to go to joint degrees
                    totalSize=ceil(time/deltat);
                    totalSize=ceil(totalSize*1.5);
                    InitialRealTimes(1,tindex)=t;
                    InitialJointPos(:,tindex)=RealVars(:,1);
                    RealTimes=[InitialRealTimes zeros(1,totalSize-20)];
                    RealJointPos=[InitialJointPos zeros(4,totalSize-20)];
                else %%index>testRangeIndex
                    self.set_joint_vars(p,200); % control motors to go to joint degrees
                    RealTimes(1,tindex)=t;
                    RealJointPos(:,tindex)=RealVars(:,1);
                end
                tindex=tindex+1;
                DataHandelingT=toc(Test)-ReadJointsT-CalculateTaskSpaceT-CalculatePositionT-setupT;
                Timers(:,tindex)=[setupT;CalculatePositionT;CalculateTaskSpaceT;ReadJointsT;DataHandelingT;outOfLoopT];
%                 thingymebober=thingymebober+1;

                % check for singularity
                self.checkSingular(0.5);
            end
%             RealTimes=RealTimes(1,1:tindex-1);
%             RealJointPos=RealJointPos(:,1:tindex-1);
%             Timers=Timers(:,1:tindex-1);
%             data=[RealTimes;RealJointPos;Timers];
            data = [];
        end
        
        function J = get_jacobian(self)
            J = self.lab2Step2.calculateJac(self.read_joint_vars(true,false,true));
        end

        %vel: the 6x1 vector including the task-space linear velocities 𝑝̇ and time = 25; % time it takes for traj in seconds
        % angular velocities 𝜔 vel: [𝑝̇;w]
        function vel=vel2fdk(self)
            jointvars=self.read_joint_vars(true,true);
            vel=self.lab2Step2.vel2fdk(jointvars(1,:)',jointvars(2,:)');
        end

        % use a cubic trajectory to move above the ball, open jaw, move to the 
        % ball's 3D position (ballPos(1),ballPos(2),10), close jaw ,move to the 
        % placePos, open jaw
        % INPUT ballPos: 2x1 vector of the ball's position in the robot's task space [X;Y]
        % INPUT placePos: 3x1 vector of where the arm should let go of the ball in the robot's task space [X;Y;Z]
        function moveBall(self, ballPos, placeBall) 
            time = 2; % time it takes for traj in seconds
            alpha = 270; % angle of EE
            home = [120 0 100 alpha];
            self.writeGripper(true); % open if closed

            TP= Traj_Planner(); % create trajectory planner instance

            % read current joint angles and convert to task space
            currJoints = self.read_joint_vars(true,false,true); 
            T = self.BaseToTipT(self.joints2fk(currJoints));
            currPos = [T(1:3,4)' -(currJoints(2)+currJoints(3)+currJoints(4)-360)]; % [x y z alpha]
            
            % 150 z height chosen randomly, doesn't matter as long as it's above ball
            finalPos = [ballPos' 100 alpha]; 

            % find traj coefficients
            coefficients = zeros(4,4);
            for col = 1:4 % [x y z alpha]
                coefficients(:,col) = TP.cubic_traj(0,time,0,0,currPos(col),finalPos(col));
            end
            %TP.plotqunticTraj(0,time,0,0,currPos(col),finalPos(col))
            
            self.run_trajectory(coefficients,time,true); % hover over [x y] location of ball

            % read current joint angles and convert to task space
            currJoints = self.read_joint_vars(true,false,true); 
            T = self.BaseToTipT(self.joints2fk(currJoints));
            currPos = [T(1:3,4)' -(currJoints(2)+currJoints(3)+currJoints(4)-360)]; % [x y z alpha]

            % approach 10 z height of ball
            finalPos = [ballPos' 5 alpha]; 

            % find traj coefficients
            for col = 1:4 % [x y z alpha]
                coefficients(:,col) = TP.cubic_traj(0,time,0,0,currPos(col),finalPos(col));
            end 
            
            self.run_trajectory(coefficients,time,true); % approach ball

            self.writeGripper(false); % grab ball

            % read current joint angles and convert to task space
            currJoints = self.read_joint_vars(true,false,true); 
            T = self.BaseToTipT(self.joints2fk(currJoints));
            currPos = [T(1:3,4)' -(currJoints(2)+currJoints(3)+currJoints(4)-360)]; % [x y z alpha]

            % set finalPos to home position
            finalPos = home; 

            % find traj coefficients
            for col = 1:4 % [x y z alpha]
                coefficients(:,col) = TP.cubic_traj(0,time,0,0,currPos(col),finalPos(col));
            end 

            self.run_trajectory(coefficients,time,true); % return to home

            % read current joint angles and convert to task space
            currJoints = self.read_joint_vars(true,false,true); 
            T = self.BaseToTipT(self.joints2fk(currJoints));
            currPos = [T(1:3,4)' -(currJoints(2)+currJoints(3)+currJoints(4)-360)]; % [x y z alpha]

            % set finalPos to designated drop off area
            finalPos = [placeBall' 330]; 

            % find traj coefficients
            for col = 1:4 % [x y z alpha]
                coefficients(:,col) = TP.cubic_traj(0,time,0,0,currPos(col),finalPos(col));
            end 

            self.run_trajectory(coefficients,time,true); % go to drop off zone

            self.writeGripper(true); % drop ball

            % read current joint angles and convert to task space
            currJoints = self.read_joint_vars(true,false,true); 
            T = self.BaseToTipT(self.joints2fk(currJoints));
            currPos = [T(1:3,4)' -(currJoints(2)+currJoints(3)+currJoints(4)-360)]; % [x y z alpha]

            % set finalPos to home position
            finalPos = home; 

            % find traj coefficients
            for col = 1:4 % [x y z alpha]
                coefficients(:,col) = TP.cubic_traj(0,time,0,0,currPos(col),finalPos(col));
            end 

            self.run_trajectory(coefficients,time,true); % return to home
        end

        % should use the code from earlier in the lab to locate the ball's actual position in task space.
        % cam: a camera object
        % out: 3xn array of n 3x1 vectors that represent a ball's 2D position in the task frame, and the ball's color represented as a number that corresponds to the color(1:Red, 2:Orange, 3:Yellow, 4:Green) [x;y;color]
        function out=findBalls(self,cam)
            [posBalls,ballColors]=lab5step3(cam); %nx2 and nx1
            ballColorsAsNum=zeros(size(ballColors));
            if(posBalls==0)
                out = 0;
                return;
            end
            for i=1:size(posBalls,1)
                xyz=self.lab2Step2.Lab5Step2(posBalls(i,1),posBalls(i,2),cam)
                xyz=self.lab2Step2.compensate4BallHeight(xyz(1:2,1)')
                posBalls(i,:)=xyz(1,1:2)
                ballColor=ballColors(i,1);
                if (ballColor=='r')
                    ballColorsAsNum(i,1)=1;
                elseif (ballColor=='o')
                    ballColorsAsNum(i,1)=2;
                elseif (ballColor=='y')
                    ballColorsAsNum(i,1)=3;
                elseif (ballColor=='g')
                    ballColorsAsNum(i,1)=4;
                end
            end
            out=[posBalls';ballColorsAsNum'];
        end

        % checks if robot is close to a singularity 
        % if not, return the determinant
        % INPUT epsilon: a small number close to zero, usually set to 0.01
        function d = checkSingular(self, epsilon)
            J = self.get_jacobian();
            d = det(J(1:3,1:3));  
            
            if(d<epsilon)
                self.stopMovement();
                error("Singularity detected.");
            end
        end

        % stops the robot's joint movement
        function stopMovement(self) 
            position = self.read_joint_vars(true,false,true);
            self.writeVelocities([0 0 0 0]);
            disp("stopped movement at:");
            disp(position);
        end

    end % end methods
end % end class 