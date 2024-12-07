classdef OurRobot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mlist=zeros(4,4,5);
        slist;
        glist=zeros(6,6,4);
        M; %home configuration
    end
    
    methods
        function self = OurRobot()
            SetUp;
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
        
        function outputArg = method1()
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
        end
    end
end

