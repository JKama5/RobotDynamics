% Goes through each ball from FindBalls() and determines based on its color, where to put the ball
%cam = Camera();
robot = Robot();
% Change to ideal positions of balls based on color
redPos = [-30 -170 5];
orangePos = [285 -125 5];
yellowPos = [285 125 5];
greenPos = [280 0 5];

disp("Press any key once ready to take a snapshot of the board");
pause;

while(true)
    % FindBalls gives you a 3xn array of n 3x1 vectors that represent a ball's 2D position 
    % in the task frame, and the ball's color represented as a number that corresponds to 
    % the color(1:Red, 2:Orange, 3:Yellow, 4:Green) [x;y;color]
    pause(1);
    foundBalls = robot.findBalls(cam);
    numberOfBalls=size(foundBalls,2);
    % stall between every iteration if no balls found
    if(foundBalls==0)
        pause(1);
    % Process each found ball   
   
    else
        for i = 1:numberOfBalls
            disp("start:");
            i
            foundBalls
            if (i>size(foundBalls,2))
                break;
            end
            ballInfo = foundBalls(:, i); % Extract the ith ball's information [X;Y;Color]
            ballPos = findBallPos(ballInfo); % ballPos: 2x1 vector of the ball's position in the robot's task space [X;Y]
            placePos = findPlacePos(ballInfo, redPos, orangePos, yellowPos, greenPos); % placePos: 3x1 vector of where the arm should let go of the ball in the robot's task space [X;Y;Z]
            robot.moveBall(ballPos',placePos');
            pause(.5);
            foundBalls = robot.findBalls(cam);
            if size(foundBalls,1)~=numberOfBalls-1 || size(foundBalls,1)==0
                break;
            end
            i
            foundBalls
            disp("end^")
        end
    end
end

 % Extract the ith ball's information [X;Y;Color]
function ballPos = findBallPos(ballInfo)
        ballPos = [ballInfo(1), ballInfo(2)];
end

% placePos: 3x1 vector of where the arm should let go of the ball in the robot's task space [X;Y;Z]
function placePos = findPlacePos(ballInfo, redPos, orangePos, yellowPos, greenPos)
    color = ballInfo(3);
    % Color Decision
    switch color
        case 1
            placePos = redPos; % Red position
        case 2
            placePos = orangePos; % Orange position
        case 3
            placePos = yellowPos; % Yellow position
        case 4
            placePos = greenPos; % Green position
        otherwise
            disp('Cannot identify color of ball');
            placePos = [NaN NaN NaN]; % Undefined position
    end
end
