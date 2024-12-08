function [t,velocity]=LSPBCalculator(totalTime,ticksPerSecond,MaxVel,deltaTheta)
    ticksPerTotalTime=round(ticksPerSecond*totalTime);
    rampTime=totalTime-(deltaTheta/MaxVel);
    accel=MaxVel/rampTime;
    t=[totalTime/ticksPerTotalTime:totalTime/ticksPerTotalTime:totalTime]';
    velocity=zeros(ticksPerTotalTime,1);
    steadyTime=totalTime-2*rampTime;
    if rampTime>=deltaTheta/2
        disp('triangle')
        accel=(2*deltaTheta)/(totalTime^2);
        for i=1:round(ticksPerTotalTime/2)
            velocity(i)=accel*t(i);
        end
        for i=round(ticksPerTotalTime/2)+1:1:ticksPerTotalTime
            velocity(i) = velocity(ticksPerTotalTime+1-i);
        end
    else if rampTime>0
        disp('trapazoid')
        i1=round(rampTime*ticksPerSecond);
        i2=round((steadyTime+rampTime)*ticksPerSecond);
        for i=1:i1
            velocity(i)=accel*t(i);
        end
        for i=i1+1:1:i2
            velocity(i)=MaxVel;
        end
        for i=i2+1:1:ticksPerTotalTime
            velocity(i)=velocity(i-1)-accel/ticksPerSecond;
        end
        % velocity((totalTime-rampTime)*resolution:1:end) = velocity(rampTime*resolution:-1:1);
    else
        disp('invalid inputs')
    end
    % stairs(t,velocity);
end