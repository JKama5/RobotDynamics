function [thetalist, success] ...
         = IKinSpace(Slist, M, T, thetalist0, eomg, ev)
% *** CHAPTER 6: INVERSE KINEMATICS ***
% Takes Slist: The joint screw axes in the space frame when the manipulator
%              is at the home position, in the format of a matrix with the
%              screw axes as the columns,
%       M: The home configuration of the end-effector,
%       T: The desired end-effector configuration Tsd,
%       thetalist0: An initial guess of joint angles that are close to 
%                   satisfying Tsd,
%       eomg: A small positive tolerance on the end-effector orientation 
%             error. The returned joint angles must give an end-effector 
%             orientation error less than eomg,
%       ev: A small positive tolerance on the end-effector linear position 
%           error. The returned joint angles must give an end-effector 
%           position error less than ev.
% Returns thetalist: Joint angles that achieve T within the specified 
%                    tolerances,
%         success: A logical value where TRUE means that the function found
%                  a solution and FALSE means that it ran through the set 
%                  number of maximum iterations without finding a solution
%                  within the tolerances eomg and ev.
% Uses an iterative Newton-Raphson root-finding method.
% The maximum number of iterations before the algorithm is terminated has 
% been hardcoded in as a variable called maxiterations. It is set to 20 at 
% the start of the function, but can be changed if needed.  
% Example Inputs:
% 
% clear; clc;
% Slist = [   0 0 0 0
%             0 1 1 1
%             1 0 0 0
%             0 -0.05825 -0.18625 -0.18625
%             0 0 0 0
%             0 0 0.02400 0.14800
%             ];
% 
% M = [   [0, 0, -1, 0.29345]; 
%         [0, 1, 0, 0]; 
%         [1, 0, 0, 0.18625]; 
%         [0, 0, 0, 1]];
% 
% T = [   [0, 0, -1, .15000]; 
%         [0, 1, 0, 0]; 
%         [1, 0, 0, 0.05000]; 
%         [0, 0, 0, 1]];
% close;
%  figure
%  hold on;
% Scale=.05;
% axis([-Scale*10 Scale*10 -Scale*10 Scale*10 -Scale*10 Scale*10])
% axis([-Scale*5 Scale*10 -Scale*10 Scale*10 -Scale*0 Scale*10])
% view([0 1 0])
% xlabel('X-axis'); 
% ylabel('Y-axis'); 
% zlabel('Z-axis'); 
% quiver3(0,0,0,Scale,0,0,'r');
% quiver3(0,0,0,0,Scale,0,'g');
% quiver3(0,0,0,0,0,Scale,'b');
% text(0,0,0,'base');
% 
% 
% 
% quiver3(M(1,4),M(2,4),M(3,4),M(1,1)*Scale,M(2,1)*Scale,M(3,1)*Scale,'r');
% quiver3(M(1,4),M(2,4),M(3,4),M(1,2)*Scale,M(2,2)*Scale,M(3,2)*Scale,'g');
% quiver3(M(1,4),M(2,4),M(3,4),M(1,3)*Scale,M(2,3)*Scale,M(3,3)*Scale,'b');
% text(M(1,4),M(2,4),M(3,4),'EE Home');
% 
% quiver3(T(1,4),T(2,4),T(3,4),T(1,1)*Scale,T(2,1)*Scale,T(3,1)*Scale,'r');
% quiver3(T(1,4),T(2,4),T(3,4),T(1,2)*Scale,T(2,2)*Scale,T(3,2)*Scale,'g');
% quiver3(T(1,4),T(2,4),T(3,4),T(1,3)*Scale,T(2,3)*Scale,T(3,3)*Scale,'b');
% text(T(1,4),T(2,4),T(3,4),'EE new');

% thetalist0 = [0; deg2rad(-45); deg2rad(45); deg2rad(-45);];
% eomg = 0.000001;
% ev = 0.00001;
% [thetalist, success] = IKinSpace(Slist, M, T, thetalist0, eomg, ev)
% 
% Output:
% thetalist =
%    1.5707
%    2.9997
%    3.1415
% success =
%     1

thetalist = thetalist0;
i = 0;
maxiterations = 200;
Tsb = FKinSpace(M, Slist, thetalist);
Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
err = norm(Vs(1: 3)) > eomg || norm(Vs(4: 6)) > ev;
while err && i < maxiterations
    thetalist = thetalist + pinv(JacobianSpace(Slist, thetalist)) * Vs;
    i = i + 1;
    Tsb = FKinSpace(M, Slist, thetalist);
    Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
    err = norm(Vs(1: 3)) > eomg || norm(Vs(4: 6)) > ev;
end
success = ~ err;
% display(i)
% display(Tsb)
rad2deg(thetalist)
end