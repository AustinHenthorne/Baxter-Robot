function [Check] = Check_NearEnough(DK_Config, Final_Config)
% Austin Henthorne MEC529 Final Project
% This function takes in the final configuration from the direct kinematics
% function which is a 4x4 matrix, and compares this matrix with the final
% configuration (4x4 matrix) taken in from the initial input from the user.
% STEP 5

% Temp Input: Check_NearEnough([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1], [1 0 0 0; 0
% 1 0 0; 0 0 1 0; 0 0 0 1]


% Isolate Roatation matrices
DK_Rotation = [DK_Config(1,1:3); DK_Config(2,1:3); DK_Config(3,1:3)];
Final_Rotation = [Final_Config(1,1:3); Final_Config(2,1:3); Final_Config(3,1:3)]; 

% Create unit quaternion from rotation matrix
Q_DK_column = rotation_to_quaternion(DK_Rotation);             % 4x1
Q_Final_column = rotation_to_quaternion(Final_Rotation);        % 4x1

% Turn the above quaternions into row vectors
Q_DK = transpose(Q_DK_column);              % 1x4
Q_Final = transpose(Q_Final_column);               % 1x4

%********** Comparison of the metrics using the phi 2 function***********
% Scalar Part of Q_DK
Q_DK_scalar = Q_DK(1,1);
% Vector Part of alpha_conj
Q_DK_vector = Q_DK(1,2:4);

% Scalar Part of Q_Final
Q_Final_scalar = Q_Final(1,1);
% Vector Part of Q_Final
Q_Final_vector = Q_Final(1,2:4);

Quat_mult = [ (Q_DK_scalar*Q_Final_scalar - dot(Q_DK_vector,Q_Final_vector)), Q_DK_scalar*Q_Final_vector + Q_Final_scalar*Q_DK_vector + cross(Q_DK_vector,Q_Final_vector) ];

mag_Quat_mult = norm(Quat_mult);

Check = sqrt(2*(1-mag_Quat_mult));

% If the check is >sqrt(2) or <0 then the rotation matrices are too far away














