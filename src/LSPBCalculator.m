function [velocity,t]=LSPBCalculator(totalTime,resolution,MaxVel,deltaTheta)
    rampTime=totalTime-(deltaTheta/MaxVel);
    accel=MaxVel/rampTime;
    t=[totalTime/resolution:totalTime/resolution:totalTime];
    velocity=zeros(resolution);
    steadyTime=totalTime-2*rampTime;
    if rampTime>=deltaTheta/2
        for i=1:resolution/2
            velocity(i)=accel*t(i);
        end
        for i=resolution/2:1:resolution
        velocity(i) = velocity(resolution/2:-1:1);
        end
    else
        for i=1:rampTime*resolution/totalTime
            velocity(i)=accel*t(i);
        end
        for i=rampTime*resolution/totalTime:(steadyTime+rampTime)*resolution/totalTime
            velocity(i)=MaxVel;
        end
        velocity((totalTime-rampTime)*resolution:1:end) = velocity(rampTime*resolution:-1:1);
    end
    
end