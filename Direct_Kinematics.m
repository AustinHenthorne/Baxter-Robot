function [gst] = Direct_Kinematics(theta)

% Austin Henthorne MEC529 Final Project
% This function takes in the Joint angles from STEP 3
% And outputs the transformation matrix
% STEP 4: Direct kinematics

% Temporary Input: Direct_Kinematics([1 1 1 1 1 1 1 ])

% This direct kinematics is hard coded for the baxter robot. 
% The link lengths, type of joints, axes description for each joint, 
% choice of points on the axis for each joint, and transformation for 
% tool frame in reference configuration (gst0)


% Joint types for the manipulator

type_joint = ['R'; 'R'; 'R'; 'R'; 'R'; 'R'; 'R'];

% Axes Description for each joint in the reference configuration

w1r = [0;0;1];
w2r = [-1/sqrt(2);1/sqrt(2);0];
w3r = [1/sqrt(2);1/sqrt(2);0];
w4r = w2r;
w5r = w3r;
w6r = w2r;
w7r = w3r;
joint_axes = [w1r w2r w3r w4r w5r w6r w7r];

% Choice of points on the axis for each joint in the reference
% configuration

q1r = [0;0; 270.35];
q2r = [0; 69; 270.35];
q3r = q2r;
q4r = [0; 433.35; 201.35];
q5r = q4r;
q6r = [0; 807.77; 201.35];
q7r = [0; 1037.29; 201.35];
q_axes = [q1r q2r q3r q4r q5r q6r q7r];

% Transformation for tool frame in reference configuration

p_st0 = [0; 1037.29; 201.35];
R_st0 = eye(3,3);
last_line = [zeros(1,3) 1];
gst0 = [R_st0 p_st0; last_line];




dim = 3;
num_of_joints = length(type_joint);
gst_temp = eye(dim+1,dim+1);
% The transformation upto each joint is stored in the matrix
% transform_upto_joint. It is a three-diemnsional 4 x 4 x n+1 matrix where n 
% is the number of joints. The first 4 x 4 matrix is the identity matrix when 
% the world frame and the base frame of the manipulator coincide. Otherwise
% it can be used to represent the base frame of the manipulator with respect 
% to a world frame not located at the base. We will use these matrices in the direct
% velocity kinematics problem.
transform_upto_joint = zeros(dim+1, dim+1, num_of_joints+1);
for i = 1:num_of_joints 
    transform_upto_joint(:,:,i) = gst_temp;
    if strcmp(type_joint(i), 'R')
        omega = joint_axes(:,i);
        q = q_axes(:,i);
        xi = [cross(-omega, q); omega];
    end
    if strcmp(type_joint,'P')
        omega = zeros(3,1);
        v = joint_axes(:,i);
        xi = [v; omega];
    end
    gst_joint_i = exp_twist(xi, theta(i));
   % gst_temp = gst_joint_i*gst_temp;
    gst_temp = gst_temp*gst_joint_i;
end
transform_upto_joint(:,:,end) = gst_temp;
gst = gst_temp*gst0;
end