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
        function T=FindTransformations(~,start,target)
            p=target(1:3,4)-start(1:3,4);
            r=target(1:3,1:3)*transpose(start(1:3,1:3));
            T=[r p;0 0 0 1];
        end

        %find the thetalist that will get the robot to the target
        %target:4x4 Transformation from origin to target
        %thetaList: 4x1 joint angles
        function thetaList=FindThetaList(target)
            thetaList=zeros(4,1);
            
        end

        %Move from startPos to endPos at maxVel speed using the movment
        %functions from robot
        %startPos: 4x4 Transformation from origin to end effector
        %endPos: 4x4 Transformation from origin to target position
        %maxVel: decimal percentage of maximum velocity
        %robot: robot class that writes to the robot
        function posControl(startPos,endPos,maxVel,robot)
            
        end
        %
    end
end


