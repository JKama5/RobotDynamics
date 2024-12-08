R=Robot();
    R.set_joint_vars([0,0,0,0],2000)
    pause(2);
    R.set_joint_vars([-90,0,0,0],4000)
    tic
while toc<4
    disp(R.vel2fdk())
    pause(.1);
end