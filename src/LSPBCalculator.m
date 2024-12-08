function [t,velocity]=LSPBCalculator(totalTime,ticksPerSecond,MaxVel,deltaTheta)
    ticksPerTotalTime=ticksPerSecond*totalTime;
    rampTime=totalTime-(deltaTheta/MaxVel);
    accel=MaxVel/rampTime;
    t=[totalTime/ticksPerTotalTime:totalTime/ticksPerTotalTime:totalTime]';
    velocity=zeros(ticksPerTotalTime,1);
    steadyTime=totalTime-2*rampTime;
    if rampTime>=deltaTheta/2
        accel=(2*deltaTheta)/(totalTime^2);
        for i=1:ticksPerTotalTime/2
            velocity(i)=accel*t(i);
        end
        for i=ticksPerTotalTime/2+1:1:ticksPerTotalTime
            velocity(i) = velocity(ticksPerTotalTime+1-i);
        end
    else
        for i=1:rampTime*ticksPerTotalTime/totalTime
            velocity(i)=accel*t(i);
        end
        for i=rampTime*ticksPerTotalTime/totalTime:1:(steadyTime+rampTime)*ticksPerTotalTime/totalTime
            velocity(i)=MaxVel;
        end
        for i=((steadyTime+rampTime)*ticksPerTotalTime/totalTime)+1:1:ticksPerTotalTime
            velocity(i)=velocity(i-1)-accel*(totalTime/ticksPerTotalTime);
        end
        % velocity((totalTime-rampTime)*resolution:1:end) = velocity(rampTime*resolution:-1:1);
    end
    stairs(t,velocity);
end