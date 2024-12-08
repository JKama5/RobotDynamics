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
            T=inv(start)*target;
        end

        %Finds the transformation from the origin to point P with an
        %orientation where the end effector is in line between the origin
        %and the point while parallel to the board
        %T: 4x4 tranformation matrix of EE when EE is at point pos
        %pos: 3x1 position vector
        function T=FindTFromPosAndAngle(~,pos)
            newX=[0;0;1];
            newZ=-unitVector([pos(1);pos(2);0]);
            newY=cross(newZ,newX);
            T=[newX newY newZ pos; 0 0 0 1];
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


