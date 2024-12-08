function [thetalist, success] = IKinBody(Blist, M, T, thetalist0, eomg, ev)
% *** CHAPTER 6: INVERSE KINEMATICS ***
% Takes Blist: The joint screw axes in the end-effector frame when the
%              manipulator is at the home position, in the format of a 
%              matrix with the screw axes as the columns,
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
% Blist = [ -1, 0, 0, 0, 0, 0;
%            0, 1, 1, 0, 1, 0;
%            0, 0, 0, 1, 0, 1;
%            0, 1.3155, 1.3155, 0, .085, 0;
%            1.445, 0, 0, 0, 0, 0;
%            0, 1.27, .175, 0, 0, 0]
% 
% M = [   [0, 0, 1, 1.4905]; 
%         [0, 1, 0, 0]; 
%         [-1, 0, 0, 1.765]; 
%         [0, 0, 0, 1]];
% 
% T =  [   [0.0321, 0.5024, 0.86, -0.305]; 
%         [0.2751, -0.8355, 0.4756, 1.1485]; 
%         [0.9609, 0.2224, -0.165, 2.3195]; 
%         [0, 0, 0, 1]];
% T =  [   [-1, 0, 0, 0]; 
%         [0.2751, -0.8355, 0.4756, 1.1485]; 
%         [0.9609, 0.2224, -0.165, 2.3195]; 
%         [0, 0, 0, 1]];
% close;
% figure
% hold on;
% Scale=.2;
% axis([-Scale*10 Scale*10 -Scale*10 Scale*10 -Scale*10 Scale*10])
% xlabel("X");
% ylabel("Y");
% zlabel("Z");
% view([0 -1 0])
% quiver3(0,0,0,Scale,0,0,'r');
% quiver3(0,0,0,0,Scale,0,'g');
% quiver3(0,0,0,0,0,Scale,'b');
% 
% 
% 
% quiver3(M(1,4),M(2,4),M(3,4),M(1,1)*Scale,M(2,1)*Scale,M(3,1)*Scale,'r');
% quiver3(M(1,4),M(2,4),M(3,4),M(1,2)*Scale,M(2,2)*Scale,M(3,2)*Scale,'g');
% quiver3(M(1,4),M(2,4),M(3,4),M(1,3)*Scale,M(2,3)*Scale,M(3,3)*Scale,'b');
% 
% quiver3(T(1,4),T(2,4),T(3,4),T(1,1)*Scale,T(2,1)*Scale,T(3,1)*Scale,'r');
% quiver3(T(1,4),T(2,4),T(3,4),T(1,2)*Scale,T(2,2)*Scale,T(3,2)*Scale,'g');
% quiver3(T(1,4),T(2,4),T(3,4),T(1,3)*Scale,T(2,3)*Scale,T(3,3)*Scale,'b');
% thetalist0 = [deg2rad(90); deg2rad(0); deg2rad(-10); 0; 0; 0;];
% eomg = 0.0001;
% ev = 0.0001;
% [thetalist, success] = IKinBody(Blist, M, T, thetalist0, eomg, ev)
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
Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
while err && i < maxiterations
    thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;
    i = i + 1;
    Tsb = FKinBody(M, Blist, thetalist);
    Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
    err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
end
success = ~ err
% display(i)
% display(Tsb)
end