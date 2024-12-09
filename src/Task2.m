clear;
close all;
R=OurRobot;
% Model(R.mlist);
Task=Task1Functions(R);
TA=Task.FindTFromPosAndAngle([.185,-.185,.185]');
TB=Task.FindTFromPosAndAngle([.185,.170,.070]');
TC=Task.FindTFromPosAndAngle([.185, 0, .240]');
figure;
title('End Effector 3D Position and Orientation')
PlotFrame(TA,true,"TA");
PlotFrame(TB,true,"TB");
PlotFrame(TC,false,"TC");
thetaListA=IKinSpace(R.slist,R.M,TA,deg2rad([-45;0;30;-30]),.00001,.00001);
thetaListB=IKinSpace(R.slist,R.M,TB,deg2rad([45;45;45;-60]),.00001,.00001);
thetaListC=IKinSpace(R.slist,R.M,TC,deg2rad([0;-45;0;10]),.00001,.00001);

ticksPerSecond=100;
timePerFirstMotion=9;
timePerSecondMotion=timePerFirstMotion;
jointVelocities=zeros(4,ticksPerSecond*20);
time=1/ticksPerSecond:1/ticksPerSecond:20;
for i=1:4
    [t,jointVelocities(i,1:ticksPerSecond*timePerFirstMotion)]=LSPBCalculator(timePerFirstMotion,ticksPerSecond,.1,thetaListC(i)-thetaListA(i));
    [t2,jointVelocities(i,ticksPerSecond*(2+timePerFirstMotion)+1:end)]=LSPBCalculator(timePerFirstMotion,ticksPerSecond,.25,thetaListB(i)-thetaListC(i));
    
end
figure;
stairs(time,jointVelocities');
xlabel('time (s)')
ylabel('velocity (rad/s)')
legend('j1','j2','j3','j4')
title('Joint Velocities Over Time')
JointPos=zeros(size(jointVelocities));
for i=1:size(jointVelocities,2)
    JointPos(:,i)=(sum(jointVelocities(:,1:i),2)/ticksPerSecond)+thetaListA;
end

figure;
plot(time,JointPos')
xlabel('time (s)')
ylabel('position (rad)')
hold on
scatter(0,thetaListA)
scatter(timePerFirstMotion,thetaListC)
scatter(timePerFirstMotion+2,thetaListC)
scatter(20,thetaListB)
legend('j1','j2','j3','j4')
title('Joint Positions Over Time')
subtitle('including target positions')
hold off

%plots what the robot will do
FollowTraj(JointPos,time,'plot');


% %% Run the robot
% FollowTraj(JointPos,time,'pos');

