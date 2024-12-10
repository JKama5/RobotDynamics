classdef realRobot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        initialTranslation = [0, 0, 36.076]; % [x, y, z]
        cached_pose = [0,0,0,0]; % Saves the last desired position sent to quintic_move()
        maxMotorCurrent = 1000 %mA
        mlist=zeros(4,4,5);
        slist;
        glist=zeros(6,6,4);
        M; %home configuration
    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = realRobot()
            SetUp;
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('cp');
            self.writeMotorState(true);

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);

            % Robot Dimensions
            self.mDim = [96.326, 130.23, 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)

            %Calculate Slist
            z=0.12800+0.05825+0.03808;
            self.slist = [   0 0 0 0
                        0 1 1 1
                        1 0 0 0
                        0 -(0.05825+.03808) -z -z
                        0 0 0 0
                        0 0 0.02400 0.14800];
            %set home config
            self.M = [   [0, 0, -1, 0.29345]; 
                    [0, 1, 0, 0]; 
                    [1, 0, 0, z]; 
                    [0, 0, 0, 1]];

            %calculate mlist
            I=eye(3);
            mp=zeros(3,1,4);
            mp(:,:,1)=[0;0.00057;0.07545];
            mp(:,:,2)=[0.00486;-0.00025;.10353+.05835+.03808];
            self.mlist(:,:,1)=[I mp(:,:,1); 0 0 0 1];
            m02=[1 0 0 .00486
                0 1 0 -.00025
                0 0 1 .10353+.05835+.03808
                0 0 0 1];
            self.mlist(:,:,2)=[I (mp(:,:,2)-mp(:,:,1)); 0 0 0 1];
            m03=[I [ 0.09370+0.02400; -0.00001; 0.00064+0.12800+0.05825+0.03808;]; 0 0 0 1];
            self.mlist(:,:,3)=[I (m03(1:3,4)-m02(1:3,4)); 0 0 0 1];
            m04=[I [0.06424+0.12400+0.02400; -0.00001; 0.00549+0.12800+0.05825+0.03808]; 0 0 0 1];
            self.mlist(:,:,4)=[I (m04(1:3,4)-m03(1:3,4)); 0 0 0 1];
            self.mlist(:,:,5)=[self.M(1:3,1:3) (self.M(1:3,4)-m04(1:3,4)); 0 0 0 1];
            TestHome=eye(4);
            for n=1:size(self.mlist,3)
                TestHome=TestHome*self.mlist(:,:,n);
            end
            %test mlist
            diff=TestHome-self.M;
            if norm(diff)<10^-7
                disp("valid mlist")
            end

            %calculate glist
            ro2b=[  0 1 0 
                    0 0 1 
                    1 0 0 ];
            n=6;
            self.glist(:,:,1)=sltoml([54860 791 0; 791 25604 0; 0 0 56910],114,ro2b,n);
            self.glist(:,:,2)=sltoml([224228.29 2135.31 30913.63; 2135.31 229056.61 5289.51; 30913.63 5289.51 54250.9],140,I,n);
            self.glist(:,:,3)=sltoml(ihalf2ifull([27859.41,140025.95,150406.23],[4186.08,-7162.89,-735.53]),118,I,n);
            self.glist(:,:,4)=sltoml(ihalf2ifull([172924.13 199140.18 267630.72],[-131.89 -23187.17 48.11]),218,I,n);
        
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
            self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
        end

        % Gets the joint's goal positions
        function goals = getJointPosGoals(self)
            goals = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
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

            % disp("time")
            % disp(time_ms)
            % disp("acc time")
            % disp(acc_time_ms)

            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            readings = self.gripper.getJointReadings();
            curr_pos = readings(1);
            if open
                desired_pos = 55;
            else
                desired_pos = -35;
            end

            % Check if we actually need to move
            if (abs(curr_pos - desired_pos) > 5)
                self.gripper.writePosition(desired_pos);
    
                % Give the gripper time to physically open/close
                pause(0.5)
            end % If we actually need to move
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
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end

        function setCurrentLimit(self, percentage)

            if percentage < 0 || percentage > 100
                error('Percentage must be between 0 and 100');
            end
            maxCurrent = self.maxMotorCurrent; % Replace with your motor's max current in mA
            scaledCurrent = (percentage / 100) * maxCurrent;
        
            self.writeCurrents(repmat(scaledCurrent, 1, 4)); % Apply to all joints
        end

        function moveToPositionWithSpeed(self, targetAngles, speedPercentage)

            self.setCurrentLimit(speedPercentage);
        

            self.writeJoints(targetAngles);
        
            % Wait for the motion to complete (adjust delay based on motion)
            % pause(5); % Example wait time, adjust based on your setup
        end


        function [thetaList, success] = FindThetaList(self, target, thetaList0)
                 
                    B = self.slist;
                    Home = self.M;
                    eomg = 1e-3;
                    ev = 1e-3;
                    [thetaList, success] = IKinBody(B, Home, target, thetaList0, eomg, ev);
                    
                    if ~success
                        error('Inverse kinematics did not converge to a solution.');
                    end
        end


        function posControl(self, startT, endT, maxVel, Tf, thetaList0)
           
            N = 100; % Number of trajectory points
            traj = CartesianTrajectory(startT, endT, Tf, N, 5); 
            
            if iscell(traj)
                numPoints = numel(traj); 
            else
                numPoints = size(traj, 3); 
            end
          
            N = min(N, numPoints);
       
            for i = 1:N
                if iscell(traj)
                    T_target = traj{i}; % For cell array
                else
                    T_target = traj(:, :, i); % For 3D array
                end
      
                T_target = double(T_target);
     
                [thetaList, success] = self.FindThetaList(T_target, thetaList0);
                if ~success
                    error('Inverse kinematics failed at trajectory point %d.', i);
                end

                thetaList0 = thetaList;
        
                self.writeJoints(rad2deg(thetaList)); 
        
                pause(Tf / N * (1 - maxVel)); 
            end
        end


       function thetaList = computeJointAngles(self, targetPos, targetOrient,thetaList0)
            % Compute the target transformation matrix
            T_target = [targetOrient, targetPos; 0, 0, 0, 1];
            disp('Target transformation matrix (T_target):');
            disp(T_target);
        
            % Solve inverse kinematics
            eomg = 1e-2; % Orientation error tolerance
            ev = 1e-2;   % Position error tolerance
            [thetaList, success] = IKinSpace(self.slist, self.M, T_target, thetaList0, eomg, ev);
            if any(abs(thetaList) > 1.54)
                thetaList0 = wrapToPi(thetaList)/2;
                disp('I enetered')
                thetaList = computeJointAngles(self,targetPos,targetOrient,thetaList0);
            end
            thetaList = wrapToPi(thetaList);

            % Ensure the result is a column vector
            thetaList = thetaList(:);
        
            % Check for success
            if ~success
                error('Inverse kinematics did not converge to a solution.');
            end
        end



        function T=FindTFromPosAndAngle(~,pos)
            newX=[0;0;1];
            newZ=-unitVector([pos(1);pos(2);0]);
            newY=cross(newZ,newX);
            T=[newX newY newZ pos; 0 0 0 1];
        end


        function outputArg = method1()
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
        end


    end % end methods
end % end class 