M=Model(false);
Calc=M.lab2step2;
point=zeros(3,3);
point(1,:)=[100,0,300];
point(2,:)=[250 0 250];
point(3,:)=[200,0, 150];
for i=1:3
    figure;
    EndEffectorLocation=point(i,:)
    Original=Calc.step2ik(point(i,:),0)
    Numeric=(Calc.numericIK(point(i,:),[0 0 0 0],true))'
    Title= '3D Stick Model of Robot Arm at: ' + string(point(i,1)) + ' ' + string(point(i,2)) + ' ' + string(point(i,3));
    title(Title);
end
%Original=Calc.step2ik(point1,0)
%Numeric=Calc.numericIK(point1,[0 0 0 0],true)