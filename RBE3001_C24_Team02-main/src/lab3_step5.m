%% config
travelTime = .016;
travelTimeMS = travelTime*1000

point1 = [80 -75 40 300]; % [x y z alpha]
point2 = [190, 0, 290, 350];
point3 = [100 65 190 340];

robot = Robot(); % create robot instance
robot.writeMotorState(true); % Write position mode
robot.set_joint_vars([0 0 0 0],5000); % send to home position
pause(5);

joints1 = robot.step2ik(point1(1:3),point1(4)); % in joint space
joints2 = robot.step2ik(point2(1:3),point2(4));
joints3 = robot.step2ik(point3(1:3),point3(4));
joints = [joints1 joints2 joints3 joints1];

%T = Traj_Planner(); % instance of Traj_Planner() class

%% Run
robot.set_joint_vars(joints1,travelTimeMS);
tic;

n = 1;
while n < 4
    index = n+(n-1)*3;

    read_joint = robot.read_joint_vars(true,false);
    pos = read_joint(1,:);
    error = norm(pos - joints(index:index+3));
    
%     pos
%     joints(index:index+3)
%     norm(pos - joints(index:index+3))

    if(error <= 1.3) % found from observation
        toc
        robot.set_joint_vars(joints(index+4:index+7),travelTimeMS);
        tic;
        n = n + 1;
    end
end

