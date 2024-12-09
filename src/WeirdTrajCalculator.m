function [t,velocity]=WeirdTrajCalculator(totalTime,ticksPerSecond,MaxVel,deltaTheta,initialVel)
    MaxVel=MaxVel*(deltaTheta/abs(deltaTheta));
    ticksPerTotalTime=round(ticksPerSecond*totalTime);
    rampTime=2*totalTime-(2*deltaTheta/MaxVel);
    accel=MaxVel/rampTime;
    t=[totalTime/ticksPerTotalTime:totalTime/ticksPerTotalTime:totalTime]';
    velocity=zeros(ticksPerTotalTime,1);
    steadyTime=totalTime-rampTime;
    if rampTime>=totalTime
        error('not trap');
        % disp('Slash');
        % if initialVel==0
        %      MaxVel=2*deltaTheta/totalTime;
        %      accel=(MaxVel/totalTime);
        %     for i=1:ticksPerTotalTime
        %         velocity(i)=accel*t(i);
        %     end
        % else
        %     PlannedDeltaTheta=initialVel*totalTime/2;
        %     if (PlannedDeltaTheta~=deltaTheta)
        %         ddt=deltaTheta-PlannedDeltaTheta;
        %         hyp=hypot(initialVel,totalTime);
        %         h=2*ddt/hyp;
        %         a=hypot(h,hyp);
        %         phi1=atan2(h,hyp/2);
        %         phi2=atan2(initialVel,totalTime);
        %         y=a*sin(phi1+phi2);
        %         x=a*cos(phi1+phi2);
        %         % accel1=
        %         % accel2=
        %         %incomplete code here
        % 
        %     else
        %         for i=1:ticksPerTotalTime
        %             MaxVel=initialVel;
        %             accel=(MaxVel/totalTime);
        %             velocity(i)=initialVel-accel*t(i);
        %         end
        %     end
        % end
    elseif rampTime>0
        disp('half-trapazoid')
        if initialVel==0
            i1=round(rampTime*ticksPerSecond);
            for i=1:i1
                velocity(i)=accel*t(i);
            end
            for i=i1+1:1:ticksPerTotalTime
                velocity(i)=MaxVel;
            end
        else
            i1=round(steadyTime*ticksPerSecond);
            for i=1:i1
                velocity(i)=MaxVel;
            end
            for i=i1+1:ticksPerTotalTime
                velocity(i)=MaxVel-accel*(t(i)-steadyTime);
            end
        end
    else
        disp('invalid inputs (weird)')
    end
end