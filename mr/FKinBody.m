function T = FKinBody(M, Blist, thetalist)
% *** CHAPTER 4: FORWARD KINEMATICS ***
% Takes M: the home configuration (position and orientation) of the
%          end-effector,
%       Blist: The joint screw axes in the end-effector frame when the 
%              manipulator is at the home position,
%       thetalist: A list of joint coordinates.
% Returns T in SE(3) representing the end-effector frame when the joints 
% are at the specified coordinates (i.t.o Body Frame).
% Example Inputs:
% 
%  clear; clc;
% Blist = [ -1, 0, 0, 0, 0, 0;
%            0, 1, 1, 0, 1, 0;
%            0, 0, 0, 1, 0, 1;
%            0, 1.3155, 1.3155, 0, .085, 0;
%            1.445, 0, 0, 0, 0, 0;
%            0, 1.27, .175, 0, 0, 0];
% 
% M = [   [0, 0, 1, 1.4905]; 
%         [0, 1, 0, 0]; 
%         [-1, 0, 0, 1.765]; 
%         [0, 0, 0, 1]];
% thetalist =     [1.9362
%     0.0569
%    -0.5588
%     1.7831
%    -1.5307
%     0.8257];
% T = FKinBody(M, Blist, thetalist)
% 
% Output:
% T =
%   -0.0000    1.0000         0   -5.0000
%    1.0000    0.0000         0    4.0000
%         0         0   -1.0000    1.6858
%         0         0         0    1.0000

T = M;
for i = 1: size(thetalist)
    T = T * MatrixExp6(VecTose3(Blist(:, i) * thetalist(i)));
end
end