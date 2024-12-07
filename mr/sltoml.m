function out=sltoml(moiAtocs,mass,ro2b,n)
    outR=transpose(ro2b)*moiAtocs*ro2b;
    outR=outR/(10^n);
    % outR=outR*10^-9;
    outP=(mass/1000)*[1 0 0;0 1 0; 0 0 1];
    out=[outR zeros(3,3); zeros(3,3) outP];
end