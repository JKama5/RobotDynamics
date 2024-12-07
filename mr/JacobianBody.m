function Jb = JacobianBody(Blist, thetalist)
% *** CHAPTER 5: VELOCITY KINEMATICS AND STATICS ***
% Takes Blist: The joint screw axes in the end-effector frame when the
%              manipulator is at the home position, in the format of a 
%              matrix with the screw axes as the columns,
%       thetalist: A list of joint coordinates.
% Returns the corresponding body Jacobian (6xn real numbers).
% Example Input:
% 
% clear; clc;
% Blist = [ -1, 0, 0, 0, 0, 0;
%            0, 1, 1, 0, 1, 0;
%            0, 0, 0, 1, 0, 1;
%            0, 1.3155, 1.3155, 0, .085, 0;
%            1.27, 0, 0, 0, 0, 0;
%            0, 1.27, .175, 0, 0, 0]
% thetalist = [2.06635101876884
%             0.331631412427870
%             -0.892889542699434
%             1.71438895750242
%             -1.65180174097943
%             0.768252101994524]
% Jb = JacobianBody(Blist, thetalist)
% 
% Output:
% Jb =
%   -0.0453    0.9950         0    1.0000
%    0.7436    0.0930    0.3624         0
%   -0.6671    0.0362   -0.9320         0
%    2.3259    1.6681    0.5641    0.2000
%   -1.4432    2.9456    1.4331    0.3000
%   -2.0664    1.8288   -1.5887    0.4000

Jb = Blist;
T = eye(4);
for i = length(thetalist) - 1: -1: 1   
    T = T * MatrixExp6(VecTose3(-1 * Blist(:, i + 1) * thetalist(i + 1)));
	Jb(:, i) = Adjoint(T) * Blist(:, i);
end
end