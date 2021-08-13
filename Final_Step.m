function [Final] = Final_Step()

% This is the Master Function, running this will give you the final answer
% Austin Henthorne and Glenn Ellis

% Declaring 4x4 transformation matrices
Final_config = [1 0 0 400; 0 cos(1) -sin(1) 500; 0 sin(1) cos(1) 400; 0 0 0 1];
Initial_1 = [1 0 0 168.91; 0 1 0 1037.29; 0 0 1 201.35; 0 0 0 1];
Initial_2 = [1 0 0 -168.91; 0 1 0 1037.29; 0 0 1 201.35; 0 0 0 1];
% Creating Dual Quaternions from the transformation matrices
A = TransMatrix_to_DualQuat(Initial_1);
B = TransMatrix_to_DualQuat(Initial_2);
C = TransMatrix_to_DualQuat(Final_config);
% Interpolating the Dual Quaternions
D1 = DualQuat_Interpolation(A, C);
D2 = DualQuat_Interpolation(B, C);
% Obtaining the Joint Angles of the final configuration
T1 = Theta_Function(D1);
T2 = Theta_Function(D2);
% Direct Kinematics
DK1 = Direct_Kinematics(T1);
DK2 = Direct_Kinematics(T2);
% Check matrices
C1 = Check_NearEnough(DK1, Final_config);
C2 = Check_NearEnough(DK2, Final_config);
% Compare Joint Angles
Compare1 = Compare_Angles(T1)
Compare2 = Compare_Angles2(T2)






