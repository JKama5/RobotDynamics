%%Setup
model = Model(false);
cam = Camera();
pos = Lab2Step2();
%% run to remove last data point
Pos=Pos(:,1:end-1)
error=error(:,1:end-1)
%% Run to clear Pos and error
Pos=[];
error=[];
AllTestOut=[];
imagePoints=[];
%% Main code
close all;

%show camera view
I = cam.getImage();
imshow(I);

%send robot to point selected on screen 
testIn = ginput(1);
robot = Robot();
    %get current position before moving (set the robot to target position manually while waiting for ginput to compare to where it actually goes
    joint_vars = robot.read_joint_vars(true,false,true); %
    testOut=model.fk(joint_vars);
GoToOut=pos.Lab5Step2(testIn(1),testIn(2),cam);
robot.set_joint_vars([0 0 0 0],2000); %go home firse
pause(2);
robot.writeGripper(true)
pause(.5);
robot.set_joint_vars(pos.step2ik(GoToOut,270),2000);
pause(2);
robot.writeGripper(false);

%determine actual end position to compare to desired end position
joint_vars = robot.read_joint_vars(true,false,true);
actualOut=model.fk(joint_vars);
%updated position and error
Pos=[Pos,GoToOut]
error=[error,(testOut-actualOut)'];
error(3,end) = error(3,end) + 10
%% Get all robot starting positions
    robot = Robot();
    model = Model(false);
    joint_vars = robot.read_joint_vars(true,false,true); %
    testOut=model.fk(joint_vars);
    AllTestOut=[AllTestOut;testOut]
%% plot test outs
figure;
I = cam.getImage();
imshow(I);
hold on;
plot(testIn(:,1),testIn(:,2),'-ob')
hold off;
figure;
plot3(AllTestOut(:,1),AllTestOut(:,2),AllTestOut(:,3),'ob')
xlim([50,175])
ylim([-137.5,137.5])
zlim([0,200])

%% Get all camera outputs and ending positions
close all;

%show camera view
I = cam.getImage();


imshow(I);
hold on;
plot(cam.IPs(:,1),cam.IPs(:,2),'-ob');
hold off;
testIn=zeros(40,2);
for (r=1:4)
    for c=1:10
        si=(c-1)*4+r;
        ei=(r-1)*10+c;
        testIn(ei,:)=cam.IPs(si,:);
    end
end

figure;
imshow(I)
hold on;
plot(testIn(:,1),testIn(:,2),'-ob')
hold off;

%% Get Pos and error experimentally for all points simultaniously
Pos = [];
error = [];
AllactualOut=[];
robot=Robot();
for i=1:size(testIn,1)
    GoToOut=pos.Lab5Step2(testIn(i,1),testIn(i,2),cam)
    robot.set_joint_vars([0 0 0 0],2000); %go home firse
    pause(2);
    robot.writeGripper(true)
    pause(.5);
    robot.set_joint_vars(pos.step2ik(GoToOut,270),2000);
    pause(2);
    robot.writeGripper(false);
    pause(.5);
    %determine actual end position to compare to desired end position
    joint_vars = robot.read_joint_vars(true,false,true);
    actualOut=model.fk(joint_vars)
    %updated position and error
    Pos=[Pos,GoToOut]
    error=[error,(AllTestOut(i,:)-actualOut)'];
    error(3,end) = error(3,end) + 10
    AllactualOut=[AllactualOut, actualOut]
end
AllactualOut=Pos-error;
AllactualOut=AllactualOut';
%%
%make a plot that shows all of the points that are covered by the current
%Pos and error data
figure;
scatter(Pos(1,1:end),Pos(2,1:end))
hold on
scatter
hold off
xlabel('X pos');
ylabel('Y Pos');

%%
figure;
plot3(AllTestOut(:,1),AllTestOut(:,2),AllTestOut(:,3),'ob')
hold on;
plot3(AllactualOut(:,1),AllactualOut(:,2),AllactualOut(:,3),'or')
quiver3(AllactualOut(:,1),AllactualOut(:,2),AllactualOut(:,3),error(1,:)', error(2,:)',error(3,:)'-8,1)
hold off;
xlabel("X");
ylabel("Y");
zlabel("Z");

%%
%for positve Y, mean of Y and X error
MeanPosError=[];
for i=1:40
    if (mod(i,10)>=5)
        MeanPosError=[MeanPosError error(:,i)];
    end
end
mean(MeanPosError')'

%%
%make a plot that shows all of the points that are covered by the current
%Pos and error data
figure;
scatter(Pos(1,1:end),Pos(2,1:end))
hold on
scatter(AllTestOut(1:end,1),AllTestOut(1:end,2))
hold off
xlabel('X pos');
ylabel('Y Pos');



%% clear and reset
%%clear all
load("error(Lab5Step2Tests).mat")
load("Pos(Lab5Step2Tests).mat")

%% Comparing error to position to see if there are any trends
TargetAngle=atan2(Pos(1,:),Pos(2,:))*180/pi;

out = sortrows([TargetAngle;Pos;error]',1).';
TargetAngle=out(1,:);
Pos=out(2:4,:);
error=out(5:7,:);

figure;
scatter(Pos(2,:),error(2,:)) %found linear trend
xlabel('Y pos');
ylabel('Y error');



figure;
scatter(Pos(1,:),error(2,:))
xlabel('X pos');
ylabel('Y error');

figure;
scatter(Pos(2,:),error(1,:)) %found linear trend
xlabel('Y pos');
ylabel('X error');

figure;
scatter(Pos(1,:),error(1,:))
xlabel('X pos');
ylabel('X error');
%% Just Negative Y
%%Input these then manually delete undesired data :/
%negYpos_error = error(:,:);
%negYpos_pos = Pos(:,:);
%posYpos_error = error(:,:);
%posYpos_pos = Pos(:,:);
%%or run bellow code to automatically sort data
negYpos_error=zeros(size(error));
posYpos_error=zeros(size(error));
negYpos_pos=zeros(size(Pos));
posYpos_pos=zeros(size(Pos));
neg_index=1;
pos_index=1;
for i=1:size(error,2)
    if Pos(2,i)<=0
        negYpos_pos(:,neg_index)=Pos(:,i);
        negYpos_error(:,neg_index)=error(:,i);
        neg_index=neg_index+1;
    else
        posYpos_pos(:,pos_index)=Pos(:,i);
        posYpos_error(:,pos_index)=error(:,i);
        pos_index=pos_index+1;
    end
end
negYpos_error=negYpos_error(:,1:neg_index-1);
negYpos_pos=negYpos_pos(:,1:neg_index-1);
posYpos_error=posYpos_error(:,1:pos_index-1);
posYpos_pos=posYpos_pos(:,1:pos_index-1);

%% Tested potential adjustments to see effectivness

coeffY=polyfit(Pos(2,:),error(2,:),1) %linear trend of Y error with respect to Y position
coeffXNew=polyfit(negYpos_pos(2,:),negYpos_error(1,:),1) 
Xconst=mean(posYpos_error(1,:))

%adjust Y error using linear trend and plot to compare with original Y error
newErrorY=error(2,:)-coeffY(2)-Pos(2,:)*coeffY(1);
figure;
scatter(Pos(2,:),[newErrorY;error(2,:)])
hold on
meanErrorY=mean(error(2,:))*ones(1,size(error,2));
meannewErrorY=mean(newErrorY)*ones(1,size(newErrorY,2));
sortedPosY=sort(Pos(2,:))
plot(sortedPosY,meanErrorY,'r',sortedPosY,meannewErrorY,'b')
hold off;
xlabel('Y pos');
ylabel('Y error');
legend('New Y Error', 'Old Y Error');

%adjust X error using linear trend and plot to compare with original X error
Pos=[negYpos_pos posYpos_pos];
error=[negYpos_error posYpos_error];

newErrorX=error(1,:)-coeffX(2)-Pos(2,:)*coeffX(1);
negYpos_newerErrorX=negYpos_error(1,:)-coeffXNew(2)-negYpos_pos(2,:)*coeffXNew(1);
posYpos_newerErrorX=posYpos_error(1,:)-Xconst;
newerErrorX=[negYpos_newerErrorX posYpos_newerErrorX];

stdErrorX=std(error(1,:))
stdNewErrorX=std(newErrorX)
stdNewerErrorX=std(newerErrorX)

stdErrorY=std(error(2,:))
stdNewErrorY=std(newErrorY)


figure;
scatter(Pos(2,:),[newerErrorX;newErrorX;error(1,:)])
hold on
meanErrorX=mean(error(1,:))*ones(1,size(error,2));
meannewErrorX=mean(newErrorX)*ones(1,size(newErrorX,2));
meanNewerErrorX=mean(newerErrorX)*ones(1,size(newerErrorX,2));
plot(Pos(2,:),meannewErrorX,'r',Pos(2,:),meanNewerErrorX,'b')
plot(sortedPosY,stdNewErrorX*ones(size(Pos(2,:))),'--r',sortedPosY,-stdNewErrorX*ones(size(Pos(2,:))),'--r')
plot(sortedPosY,stdNewerErrorX*ones(size(Pos(2,:))),'--b',sortedPosY,-stdNewerErrorX*ones(size(Pos(2,:))),'--b')
plot(Pos(2,:),meanErrorX,'Color','#EDB120')
% errorbar(Pos(2,:),newerErrorX,-stdNewerErrorX*ones(size(newerErrorX)),stdNewerErrorX*ones(size(newerErrorX)),'vertical', 'b','Marker','none','LineStyle','none');
% errorbar(Pos(2,:),newErrorX,-stdNewerErrorX*ones(size(newErrorX)),stdNewerErrorX*ones(size(newErrorX)),'vertical', 'r','Marker','none','LineStyle','none'); % Error bars with standard deviation as width
hold off;
xlabel('Y pos');
ylabel('X error');
legend('Newer X Error', 'New X Error', 'Old X Error');

% calculate norms of error before and after fixes to get numeric evidence that changes improved error
normsPreFix=[];
normsPostFix=[];
normsPostNewFix=[];
for i=1:size(error,2)
    normsPreFix(i)=norm(error(1:2,i));
    normsPostFix(i)=norm([newErrorX(i);newErrorY(i)]);
    normsPostNewFix(i)=norm([newerErrorX(i);newErrorY(i)]);
end

prefixMeanError=mean(normsPreFix)
PostFixmeanError=mean(normsPostFix)
PostNewFixmeanError=mean(normsPostNewFix)
figure;
scatter(Pos(2,:),[normsPostNewFix;normsPostFix;normsPreFix])
hold on
prefixMeanErrorPlot=prefixMeanError*ones(1,size(error,2));
postFixMeanErrorPlot=PostFixmeanError*ones(1,size(newErrorX,2));
PostNewFixMeanError=PostNewFixmeanError*ones(1,size(newerErrorX,2));

plot(Pos(2,:),postFixMeanErrorPlot,'r',Pos(2,:),PostNewFixMeanError,'b')
plot(Pos(2,:),prefixMeanErrorPlot, 'Color','#EDB120')
hold off;
xlabel('Y pos');
ylabel('error (norm)');
legend('Newer Error', 'New Error', 'Old Error');
%% Angles
TargetAngle=atan2(Pos(1,:),Pos(2,:))*180/pi;

out = sortrows([TargetAngle;Pos;error]',1).';
TargetAngle=out(1,:);
Pos=out(2:4,:);
error=out(5:7,:);

ActualAngle=atan2(Pos(1,:)+error(1,:),Pos(2,:)+error(2,:))*180/pi;


ActualAngleImproved=atan2(Pos(1,:)+newerErrorX,Pos(2,:)+error(2,:))*180/pi;

%ActualAngle=ActualAngleImproved;

AngleError=ActualAngle-TargetAngle;

targetRadius=sqrt(Pos(1,:).^2+Pos(2,:).^2);

%error(2,:)=newerErrorX;

actualRadius=sqrt(((Pos(1,:)+error(1,:)).^2)+((Pos(2,:)+error(2,:)).^2));
radiusError=targetRadius-actualRadius;

% figure;
% plot(TargetAngle,targetRadius,LineStyle="none",Marker="o")
% hold on;
% plot(TargetAngle,actualRadius,LineStyle="none",Marker="o");
% hold off;



%plot radius Error vs Target angle
    figure;
    plot(TargetAngle,radiusError,LineStyle="none",Marker="o");

    %get line of best fit
    coefficents=polyfit(TargetAngle,radiusError,2);
    lineOfBestFitRadiusError=coefficents(1)*TargetAngle.^2+coefficents(2)*TargetAngle+coefficents(3);
    
    %plot line of best fit
    hold on
    plot(TargetAngle,lineOfBestFitRadiusError,Marker="none",LineStyle="-");
    hold off
    xlabel('Target Angle (degrees)')
    ylabel('Radius Error (degrees)')
    ylim([-15,15]);


%plot Angle Error vs Target angle
    figure
    plot(TargetAngle,AngleError,LineStyle="none", Marker="o");

    %get line of best fit
    coefficents=polyfit(TargetAngle,AngleError,2);
    lineOfBestFitAngleError=coefficents(1)*TargetAngle.^2+coefficents(2)*TargetAngle+coefficents(3);

    %plot line of best fit
    hold on
    plot(TargetAngle,lineOfBestFitAngleError,Marker="none",LineStyle="-");
    hold off
    xlabel('Target Angle (degrees)')
    ylabel('Angle Error (mm)')
    ylim([-5,5])

%%Improved Angles and radii
ActualAngleImproved=atan2(Pos(1,:)+newerErrorX,Pos(2,:)+newErrorY)*180/pi;
AngleErrorImproved=ActualAngleImproved-TargetAngle;

actualRadiusImproved=sqrt(((Pos(1,:)+newerErrorX).^2)+((Pos(2,:)+error(2,:)).^2));
radiusErrorImproved=targetRadius-actualRadiusImproved;

%plot Improved radius Error vs Target angle
    figure;
    plot(TargetAngle,radiusErrorImproved,LineStyle="none",Marker="o");

    %get line of best fit
    coefficents=polyfit(TargetAngle,radiusErrorImproved,2);
    lineOfBestFitRadiusError=coefficents(1)*TargetAngle.^2+coefficents(2)*TargetAngle+coefficents(3);
    
    %plot line of best fit
    hold on
    plot(TargetAngle,lineOfBestFitRadiusError,Marker="none",LineStyle="-");
    hold off
    
    xlabel('Target Angle (degrees)')
    ylabel('Improved Radius Error (mm)')
    ylim([-15,15]);


%plot Improved Angle Error vs Target angle
    figure
    plot(TargetAngle,AngleErrorImproved,LineStyle="none", Marker="o");

    %get line of best fit
    coefficents=polyfit(TargetAngle,AngleErrorImproved,2);
    lineOfBestFitAngleError=coefficents(1)*TargetAngle.^2+coefficents(2)*TargetAngle+coefficents(3);

    %plot line of best fit
    hold on
    plot(TargetAngle,lineOfBestFitAngleError,Marker="none",LineStyle="-");
    hold off

    xlabel('Target Angle (degrees)')
    ylabel('Improved Angle Error (degrees)')
    ylim([-5,5])

%%Improved with respect to Y
    %plot Improved radius Error vs Target Y Position
        figure;
        plot(Pos(2,:),radiusErrorImproved,LineStyle="none",Marker="o");
        
        xlabel('Target Y Position(mm)')
        ylabel('Improved Radius Error (mm)')
        ylim([-15,15]);
    
    %plot Improved Angle Error vs Target angle
        figure
        plot(Pos(2,:),AngleErrorImproved,LineStyle="none", Marker="o");
    
        xlabel('Target Y Position(mm)')
        ylabel('Improved Angle Error (degrees)')
        ylim([-5,5])
%%


improvedAngleError=ActualAngleImproved-TargetAngle;
figure;
plot3(Pos(1,:),Pos(2,:),AngleError,'r',LineStyle="none",Marker="o");
hold on
plot3(Pos(1,:),Pos(2,:),improvedAngleError,'b',LineStyle="none",Marker="o");
hold off

figure
plot(TargetAngle,AngleError,LineStyle="none", Marker="o");
hold on
plot(TargetAngle,improvedAngleError,LineStyle="none", Marker="o");
hold off;
%%

%%
save("Pos(Lab5Step2Tests).mat","Pos");
save("error(Lab5Step2Tests).mat","error");
