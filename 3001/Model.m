% sudo chmod a+rw ttyUSB0 
classdef Model
    properties
        robot   %the instance of Robot or Lab2Step2 used to preform transformations 
        lab2step2 %used for faster calculations when using Robot
    end

    methods
        % Constructor:
        % withRobot:    if true sets robot to an instance of Robot() (must be
        %                   connected to the robot to function)
        %               if false sets robot to an instance of
        %                   Lab2Step2() which allows Model to work
        %                   without the robot plugged in
        function self = Model(withRobot)
            if withRobot
                self.robot=Robot();
                self.lab2step2=Lab2Step2;    %create a lab2step2 to do lab2step2 functions faster (since Robot just calls lab2step2 anyway)
            else
                self.robot=Lab2Step2(); 
                self.lab2step2=self.robot; %if you aren't using the robot, make the robot the same element as lab2 step 2.
            end

        end
        
        % Calculate the position of the end effector
        % EEPos= 4x1 matrix of the end effector position [X;Y;Z;1]
        %         [-999 -999 -999 -999]
        function EEPos=ee_pos(self)
            if isa(self.robot,'Lab2Step2')==true
                EEPos=[-999 -999 -999, -999];
                error('Trying to plot robot when not connected to robot');
            else
                JointVar=self.robot.read_joint_vars(true,false);%get joint position [2x4] [X,Y,Z,1;X',Y',Z',1]
                T_end=self.robot.BaseToTipT(self.robot.joints2fk(JointVar(1,:)));%calculate transformation matrix from base to end effector using position component of the jointReadings 
                EEPos=T_end*[0;0;0;1];
            end

        end
        
        %Preforms the forward kinimatics to get from joint angles to
        %   position of end effector (most of the code was pulled out of
        %   plot_arm
        %p: 1x3
        function p=fk(self,jointVarDeg)
%             AxisSize=50;%the size of the frame axes' lines
            compTransform = self.robot.joints2fk(jointVarDeg);%converts the joint positions into the 4x4x4 array of the 4 4x4 transformation matrices for each joint
            sz = size(compTransform);

            %set the initial sizes and values of all the matrices
            X = zeros(1, 4);%1x5 array storing the x position of each frame
            Y = zeros(1, 4);%1x5 array storing the y position of each frame
            Z = zeros(1, 4);%1x5 array storing the z position of each frame
            P = zeros(4,1,sz(3));% a sz(3)x(4x1) array storing all of the 4x1 position vectors ([X;Y;Z;1]) for each joint 
            P(4,:,:)=1;%set row 4 to 1
%             Axis = zeros(4,3,sz(3));%a sz(3)x(4x3) array storing the vectors used to create the axes at each reference frame 
%                                         % a (4x1) position vector for each
%                                         %   axes for each frame 
%             Axis(4,:,:)=1;%set row 4 of each position vector to 1  
            
            %Calculate the tranformation matrix from the origin ([0 0 0 1])
            %   to each reference from and use that transformation matrix to calculate all of the values needed to make the plot 
            for i = 1:sz(3)
                if i==1
                    T=compTransform(:,:,i);%calculate the tranformation matrix for frame 1 (just the first tranformation matrix from compTransform())
                else 
                    T = self.robot.BaseToTipT(compTransform(:,:,1:i));%calculate the transformationn matrix from the origin to frame i (aside from frame 1)
                end
                P(:,i)=T*[0;0;0;1];%calculate position vector for this frame by multiplying the transformation matrix from the origin to that frame with the origin position vector 
%                 Axis(:,1,i)=T*[AxisSize;0;0;1];%calculate the end position vector ([X,Y,Z,1]) of the X axis for this frame
%                 Axis(:,2,i)=T*[0;AxisSize;0;1];%calculate the end position vector ([X,Y,Z,1]) of the Y axis for this frame
%                 Axis(:,3,i)=T*[0;0;AxisSize;1];%calculate the end position vector ([X,Y,Z,1]) of the Z axis for this frame
                
                %Convert from the position vector to individual x,y,z
                %   values to conform with the format already being used and improve readibility 
                X(i) = P(1,1,i);
                Y(i) = P(2,1,i);
                Z(i) = P(3,1,i);
                
                %Convert from the end point of each vector to the x,y,and z
                % distance from that specific frame of each vector ie
                % convert to the format needed for the quiver3() function
%                 Axis(1,:,i)=Axis(1,:,i)-X(i);
%                 Axis(2,:,i)=Axis(2,:,i)-Y(i);
%                 Axis(3,:,i)=Axis(3,:,i)-Z(i);  
            end
            
%             fk=T;%set the output to the last transformation matrix calculated (Base to tip)
            % Plot the robot arm
%             plot3(X, Y, Z, '-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'r');
%             
%             % Set the plot labels and title
%             xlabel('X (mm)');
%             ylabel('Y (mm)');
%             zlabel('Z (mm)');
%             title('3D Stick Model of Robot Arm');
%             
%             % Set the plot limits
%             axis equal;
%             Plotsize=sum(self.robot.mDim(2:end))+AxisSize; %the maximum X or Y distance the robot can reach + the size of the frame axes
%             xlim([-Plotsize, Plotsize]);
%             ylim([-Plotsize, Plotsize]);
%             zlim([0, Plotsize+self.robot.mDim(1)]);%the maximum height the robot can reach (includes the first link)
%             
%             % coordinate frames (axes) at each joint
%             for i = 1:size(compTransform, 3)
%                 hold on;
%                 % Creating a small axes graphic for each frame 
%                 quiver3(X(i), Y(i), Z(i), Axis(1,1,i), Axis(2,1,i), Axis(3,1,i), 'g'); % X-axis
%                 quiver3(X(i), Y(i), Z(i), Axis(1,2,i), Axis(2,2,i), Axis(3,2,i), 'm'); % Y-axis
%                 quiver3(X(i), Y(i), Z(i), Axis(1,3,i), Axis(2,3,i), Axis(3,3,i), 'b'); % Z-axis
%             end
% 
%             grid on;%turn on the grid
%             view(3);%set plot orientation
%             hold off;
            p=P(1:3,1,sz(3)).';5;%pull out the final position values [Xee,Yee,Zee]
        end

        %Plots a stick model of the robot in 3D based on the robot's joint
        %   positions 
        %JointVarDeg: a 1x4 array of the robot's joint positions
        function fk=plot_arm(self,jointVarDeg)
            
            AxisSize=50;%the size of the frame axes' lines
            compTransform = self.robot.joints2fk(jointVarDeg);%converts the joint positions into the 4x4x4 array of the 4 4x4 transformation matrices for each joint
            sz = size(compTransform);

            %set the initial sizes and values of all the matrices
            R=zeros(3,3,sz(3));
            X = zeros(1, 4);%1x5 array storing the x position of each frame
            Y = zeros(1, 4);%1x5 array storing the y position of each frame
            Z = zeros(1, 4);%1x5 array storing the z position of each frame
            P = zeros(4,1,sz(3));% a sz(3)x(4x1) array storing all of the 4x1 position vectors ([X;Y;Z;1]) for each joint 
            P(4,:,:)=1;%set row 4 to 1
            Axis = zeros(4,3,sz(3));%a sz(3)x(4x3) array storing the vectors used to create the axes at each reference frame 
                                        % a (4x1) position vector for each
                                        %   axes for each frame 
            Axis(4,:,:)=1;%set row 4 of each position vector to 1  
            
            %Calculate the tranformation matrix from the origin ([0 0 0 1])
            %   to each reference from and use that transformation matrix to calculate all of the values needed to make the plot 
            for i = 1:sz(3)
                if i==1
                    T=compTransform(:,:,i);%calculate the tranformation matrix for frame 1 (just the first tranformation matrix from compTransform())
                else 
                    T = self.robot.BaseToTipT(compTransform(:,:,1:i));%calculate the transformationn matrix from the origin to frame i (aside from frame 1)
                end
                P(:,i)=T*[0;0;0;1];%calculate position vector for this frame by multiplying the transformation matrix from the origin to that frame with the origin position vector 
                Axis(:,1,i)=T*[AxisSize;0;0;1];%calculate the end position vector ([X,Y,Z,1]) of the X axis for this frame
                Axis(:,2,i)=T*[0;AxisSize;0;1];%calculate the end position vector ([X,Y,Z,1]) of the Y axis for this frame
                Axis(:,3,i)=T*[0;0;AxisSize;1];%calculate the end position vector ([X,Y,Z,1]) of the Z axis for this frame
                
                %Convert from the position vector to individual x,y,z
                %   values to conform with the format already being used and improve readibility 
                X(i) = P(1,1,i);
                Y(i) = P(2,1,i);
                Z(i) = P(3,1,i);
                
                %Convert from the end point of each vector to the x,y,and z
                % distance from that specific frame of each vector ie
                % convert to the format needed for the quiver3() function
                Axis(1,:,i)=Axis(1,:,i)-X(i);
                Axis(2,:,i)=Axis(2,:,i)-Y(i);
                Axis(3,:,i)=Axis(3,:,i)-Z(i);
            end
            
            fk=T;%set the output to the last transformation matrix calculated (Base to tip)
            % Plot the robot arm
            plot3(X, Y, Z, '-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'r');
            
            % Set the plot labels and title
            xlabel('X (mm)');
            ylabel('Y (mm)');
            zlabel('Z (mm)');
            title('3D Stick Model of Robot Arm');
            
            % Set the plot limits
            axis equal;
            Plotsize=sum(self.robot.mDim(2:end))+AxisSize; %the maximum X or Y distance the robot can reach + the size of the frame axes
            xlim([-Plotsize, Plotsize]);
            ylim([-Plotsize, Plotsize]);
            zlim([0, Plotsize+self.robot.mDim(1)]);%the maximum height the robot can reach (includes the first link)

            % coordinate frames (axes) at each joint
            for i = 1:size(compTransform, 3)
                hold on;
                % Creating a small axes graphic for each frame 
                quiver3(X(i), Y(i), Z(i), Axis(1,1,i), Axis(2,1,i), Axis(3,1,i), 'g'); % X-axis
                quiver3(X(i), Y(i), Z(i), Axis(1,2,i), Axis(2,2,i), Axis(3,2,i), 'm'); % Y-axis
                quiver3(X(i), Y(i), Z(i), Axis(1,3,i), Axis(2,3,i), Axis(3,3,i), 'b'); % Z-axis
            end

            grid on;%turn on the grid
            view(240,25);%set plot orientation
            %hold off;
        end
    end
end