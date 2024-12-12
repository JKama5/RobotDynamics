%A class that contains all of the Lab2-Step2 code from Robot.m but works so
% that that code can be used without being pluged into the robot
classdef Lab2Step2
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        defaultTravelTime = 5000;
        Jac;
        step2;
    end
    methods
        function self = Lab2Step2()
            load('SymbolicJ.mat','J');
            self.Jac=J;
            load('SymbolicIK.mat', 'q');
            self.step2 = q; % FIX LATER?
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            %self.writeMode('p');
            %self.writeMotorState(true);
            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            %self.writeTime(2);
            % Robot Dimensions
            self.mDim = [96.326, 130.23, 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
        end
        function dhTransform = dh2mat(self,dhArr)
            theta = dhArr(1);
            d = dhArr(2);
            a = dhArr(3);
            alpha = dhArr(4);
            dhTransform = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
                sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
                0 sin(alpha) cos(alpha) d
                0 0 0 1];
        end
        % input dhMat [nx4] array corresponding to the n rows of the full DH parameter table,
        % rows take form [θ d a α]
        % return compTransform [4x4]xn array where "n" is the joint number, and each joint gets
        % a 4x4 homogenous matrix that gives the global orientation and position
        function compTransform = dh2fk(self,dhMat)
            sz = size(dhMat); % returns # row,# column of dhMat
            if ~isnumeric(dhMat)
                compTransform = sym('empty',[4,sz(2),sz(1)]);% set the size of compTransform as symbolic
            else
                compTransform = zeros(4, sz(2),sz(1)); % set the size of compTransform
            end
            % for i = 1:sz(1,1) % iterate for loop for num of rows in dhMat
            % column_index = i+(i-1)*3; % indexes through compTransform by multiples of 4
            % compTransform(:,column_index:column_index+3) = self.dh2mat(dhMat(i, :)); % saves to compTransform
            % end
            for i = 1:sz(1)% itterate through every row of the dh table
                compTransform(:,:,i) = self.dh2mat(dhMat(i,:));% runs dh2mat on each row of the dh table and saves the resulting transformation matricies to CompTransformation
            end
        end

        % input jointVar [1x4] array of the joint variables in degrees
        % return compTransform [4x4]xn array where "n" is the joint number, and each joint gets
        % a 4x4 homogenous matrix that gives the global orientation and position
        function compTransform = joints2fk(self, jointVarDeg)
            compTransform = []; % initialize
            %DH table: Theta (radians) d (mm) a (mm) alpha (radians)
            if isnumeric(jointVarDeg)
                jointVar = deg2rad(jointVarDeg); % Convert input from degrees to radians
                DHTable = [ jointVar(1) self.mDim(1) 0 -pi/2 %Link 0 & 1 (L0 & L1 were combined in mDim(1) by starter code)
                    jointVar(2)-atan2(self.mOtherDim(1),self.mOtherDim(2)) 0 self.mDim(2) 0 %Link 2
                    jointVar(3)+atan2(self.mOtherDim(1),self.mOtherDim(2)) 0 self.mDim(3) 0 %Link 3
                    jointVar(4) 0 self.mDim(4) 0]; %Link 4
            else
                    jointVar=jointVarDeg*pi/180;
                    jointvars=formula(jointVar);
                    DHTable = [ jointvars(1) self.mDim(1) 0 -pi/2 %Link 0 & 1 (L0 & L1 were combined in mDim(1) by starter code)
                        jointvars(2)-atan2(self.mOtherDim(1),self.mOtherDim(2)) 0 self.mDim(2) 0 %Link 2
                        jointvars(3)+atan2(self.mOtherDim(1),self.mOtherDim(2)) 0 self.mDim(3) 0 %Link 3
                        jointvars(4) 0 self.mDim(4) 0];
            end
            compTransform = self.dh2fk(DHTable); % plug DH table into dh2fk to get a 4x4 homogenous Transformation matrix that gives the global orientation and position
        end
        %input: [4x4]xn array output by dh2fk (array of all the individual
        % transformation matrices
        %output: [4x4] array of the resulting transformation matrix
        function T = BaseToTipT(self, in)
            sz=size(in);
            if (size(sz,2)==3)
                T=in(:,:,1);
                for i = 2:sz(3)
                    T=T*in(:,:,i);
                end
            else
                T=in;
            end
        end
        %% Lab 3
        % preform inverse kinimatics to get from desired position and
        % orientation to joint angles =
        % p: 1x3 vector of desired position: [x,y,z]
        % a: angle the end effector makes with the x axis mesured from a
        % line parrallel to the x axis going through joint 4 to the
        % underside of the end effector
        %q: 1x4 vector of the 4 joint angles: q1, q2, q3, q4
        % joint limits: q1:[-90,90] q2: [-90. 90] q3: [-90,80]
        % q4:[-90, 90]
        function q = step2ik(self,p,a)
            a=deg2rad(a);%convert all input angles to rad (all calculations done with radians)
            %change variable names to convention in work for easier
            % readibility and shorter variable names
            L1=self.mDim(1);
            L2=self.mDim(2);
            L3=self.mDim(3);
            L4=self.mDim(4);
            %calculate q1 and the equivilent x mesurment if the pose was
            % rotated to the zx plane (allows for 2d calcualtions)
            Xssq=p(1)^2+p(2)^2;
            Xs=sqrt(Xssq);
            q(1)=pi/2-atan2(p(1),p(2));
            %perform calculation in the equivilent xz plane to find q2-4
            Zs=p(3)-L1;
            LAsq=((Zs^2)+(Xs^2)); %LA^2 (faster calcualtions)
            LA=(Xs/abs(Xs))*sqrt(LAsq);
            Phi1=atan2(p(3)-L1,Xs);
            Phi10=atan2(24,128);
            if a>pi
                Phi2=a-pi;
            else
                Phi2=a+pi;
            end
            Phi6=3*pi/2-a;
            LBsign=(a-pi)/abs(a-pi);
            LB=LBsign*abs(L4*sin(Phi2));
            LC=L4*cos(Phi2);
            L5sq=(Xs+LC)^2+(p(3)-L1+LB)^2;%L5^2 (faster calcualtions)
            L5=(L5sq)^(1/2);
            Phi3=acos((L5sq+L2^2-self.mDim(3)^2)/(2*L5*L2));
            Phi4=acos((L2^2+L3^2-L5sq)/(2*L2*L3));
            Phi11=atan2(Zs+LB,Xs+LC);
            Phi5sign=(Phi11-Phi1)/abs(Phi11-Phi1); %wether or not L5 is abovce LA (+) or if L5 is bellow LA (-) which impacts Theta2 calculations
            Phi5=acos((LAsq+L5sq-L4^2)/(2*LA*L5))*Phi5sign;
            Phi7=pi-Phi1-Phi3-Phi4-Phi5;
            Phi8=pi-Phi7;
            Phi9=Phi8-pi/2;
            %Calculate thetas (joint variables)
            q(2)=pi/2-Phi1-Phi3-Phi5-Phi10;
            q(3)=pi/2-Phi4+Phi10;
            q(4)=Phi6+Phi9;
            if q(4)>3*pi/2
                q(4)=q(4)-2*pi;
            end 
            
            q=rad2deg(q);%convert all output angles back to degrees
%             Phi1=rad2deg(Phi1);
%             Phi2=rad2deg(Phi2);
%             Phi3=rad2deg(Phi3);
%             Phi4=rad2deg(Phi4);
%             Phi5=rad2deg(Phi5);
%             Phi6=rad2deg(Phi6);
%             Phi7=rad2deg(Phi7);
%             Phi8=rad2deg(Phi8);
%             Phi9=rad2deg(Phi9);
%             Phi10=rad2deg(Phi10);
            % if q(1)<-90 |q(2)<-90|q(3)<-90|q(4)<-90| q(1:2)>90 | q(3)>80 | q(4)>118 | ~isreal(q)
            %     values=string(q(1))+' '+string(q(2))+' '+' '+string(q(3))+' '+' '+string(q(4));
            %     msg = 'cant reach position! angles are:'+values;
            %     error(msg);
            %     %disp(msg);
            % end  
        end

        function q = step2ik2(self,p,a)
            a=a*pi/180;
            q=zeros(1,4);
            L1=self.mDim(1);
            L2=self.mDim(2);
            L3=self.mDim(3);
            L4=self.mDim(4);
            L2sq=L2^2;
            L3sq=L3^2;

            q(1)=atan2(p(2),p(1));
            Xstar=sqrt(p(1)^2+p(2)^2);
            
            Phi1=0.18534795; %arctan(24/128)
            Beta=2*pi-a;
            XJ4Star=Xstar-L4*cos(a);
            ZJ4Star=p(3)-L4*sin(a);
            Phi2=atan2(ZJ4Star-L1,XJ4Star);
            L5sq=XJ4Star^2+(ZJ4Star-L1)^2;
            Phi3=acos((L5sq+L2sq-L3sq)/(2*sqrt(L5sq)*L2));
            Phi4=acos((L3sq+L2sq-L5sq)/(2*L3*L2));
            q(2)=pi/2 -Phi1-Phi2-Phi3;
            q(3)=pi/2-Phi4+Phi1;
            q(4)=Beta-q(2)-q(3);
            q(4)=mod(q(4) + pi, 2*pi) - pi;

            q=q*180/pi;
        end

        %calculate the Jacobian based on joint angles and importing the
        % symbolic jacobian calculated in the SolveJacobianSymbolicaly.mlx
        %thisJ: the numeric jacobian
        %JointAngles: a 1x4 or 4x1 matrix of the 4 joint angles
        function thisJ=calculateJac(self,JointAngles)
            sym t ;
            syms Theta1(t) Theta2(t) Theta3(t) Theta4(t);%make symbolic values of theta (motor angle position inputs)
            thisJ=subs(self.Jac,[Theta1(t),Theta2(t),Theta3(t),Theta4(t)],JointAngles);
            thisJ=double(thisJ);
        end
        %JointVel: 4x1 array of instantanious joint velocity
        %JointAngles: 4x1 array of joint Angles
        %vel: the 6x1 vector including the task-space linear velocities 𝑝̇ and
        % angular velocities 𝜔: [𝑝̇;w]
        function vel=vel2fdk(self, JointAngles,JointVel)
            J=self.calculateJac(JointAngles');
            vel=J*JointVel;
        end

        % Calcualte the inverse kinimatics numerically 
        % jointAngles: 1x4 array of the required joint angles to get to the end effector position specified in EEPos 
        % EEFinalPos: 1x3 desired final end effector position (orientation is (0,0,0)
        % InitialJointAngles: 1x4 array of the initial points of the joints
        function jointAngles=numericIK(self,EEFinalPos, InitialJointAngles,plot)
            model=Model(false);
            if (plot==true)
                clf;
                model.plot_arm(InitialJointAngles);
                plot3(EEFinalPos(1),EEFinalPos(2),EEFinalPos(3),'cx','MarkerSize', 5);
                drawnow;
                pause(.5);
            end
            J=self.calculateJac(InitialJointAngles);
            lambda=0.1;
            BtoT=self.BaseToTipT(self.joints2fk(InitialJointAngles));
            EEInitial=BtoT(1:3,4);
            EEFinal=[EEFinalPos';0;0;0];
            deltaQ=J'*pinv(J*J'+lambda^2*eye(6)) * (EEFinal-[EEInitial;0;0;0]);
            jointAngles=InitialJointAngles'+deltaQ;
            while norm(deltaQ)>.1
                BtoT=self.BaseToTipT(self.joints2fk(jointAngles'));
                EEInitial=BtoT(1:3,4);
                EEFinal=[EEFinalPos';0;0;0];
                deltaQ=J'*pinv(J*J'+lambda^2*eye(6)) * (EEFinal-[EEInitial;0;0;0]);
                jointAngles=jointAngles+deltaQ;
                if (plot==true)
                    clf;
                    model.plot_arm(jointAngles');
                    plot3(EEFinalPos(1),EEFinalPos(2),EEFinalPos(3), 'cx','MarkerSize', 5);
                    drawnow;
                    pause(.1);
                end
            end
            hold off;
            jointAngles=jointAngles+deltaQ;
        end

        function opos = Lab5Step2(self,x,y,camera1)
            extrensics = camera1.getCameraPose();
            poscheckerboard =  pointsToWorld(camera1.cam_IS,extrensics(1:3,1:3),extrensics(1:3,4),[x,y]);
            
            worldtocheckerboardmatrix =[ 0  1   0   75;
                                         1  0   0   -110;
                                         0  0   -1  0;
                                         0  0   0   1];

%             worldtocheckerboardmatrix =[ 0  1   0   81;      % -6.6022
%                                          1  0   0   -87.5;    %-22.8156
%                                          0  0   -1  0;
%                                          0  0   0   1];


            postoworldframe = (worldtocheckerboardmatrix)*[poscheckerboard,0,1]';
            opos = [postoworldframe(1:2);10];

%             %adjust based on error having a linear trend with respect to Y
%             %   position 
%           
%             
            if (opos(2)<5)
                opos(1)=opos(1)-0.12626-0.08619*opos(2);
            else
                opos(1)=opos(1)-0.731621033651953;
            end
            opos(2)=opos(2)+2.68106161162923-0.0756410112771768*opos(2);
            


            %adjust based on end effector not reaching out far enough but
            %   having the correct angle
            theta=atan2(opos(2),opos(1));
            opos(1)=opos(1)-5*cos(theta);
            opos(2)=opos(2)-5*sin(theta);
% 
        end

        % Converts from the observed position of the balls with respect to
        %   the world frame when treating them as flat 2D elipses on the checkerboard, to their actual 3D position in space with respect to the world fame (F0)
        % Pout: 1x3 array of [X,Y,Z] real 3D position of the center of the balls with respect to the world frame (F0)
        % Pin:  1x2 array of [x,y] 2D observed position of the balls with respect to the world frame when treating them as flat 2D elipses on the checkerboard
        function Pout = compensate4BallHeight(self,Pin)
            r=10; %ball radius in mm
            h=173; %height of lens in mm (from board)
            Cx=360;%rogh measurment
            Pout=zeros(1,3);
            Pout(3)=r;
            phi=atan2(Cx-Pin(1),Pin(2));
            A=sqrt(Pin(2)^2+(Cx-Pin(1))^2);
            theta=atan2(h,A);
            xstar=r/tan(theta);
            deltaX=xstar*sin(phi);
            deltaY=xstar*cos(phi);
            Pout(1)=Pin(1)+deltaX;
            Pout(2)=Pin(2)-deltaY;
        end
    end %methods
end%Class
