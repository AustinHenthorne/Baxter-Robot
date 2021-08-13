function [DQ] = TransMatrix_to_DualQuat(R)
% Austin Henthorne MEC529 Final Project
% This function takes in a 4x4 transformation matrix as an input 
% And outputs a dual quaternion
% The output is the dual quaternion in the form of 1x8 matrix
% STEP 1

% Translation Vector
Translation = [R(1,4) R(2,4) R(3,4) R(4,4)];

% To solve for all the q^2's
A= [1 1 1 1;
    1 -1 -1 1;
    -1 1 -1 1;
    -1 -1 1 1];

B= [R(1,1);R(2,2);R(3,3);1];
C=.25*A*B;
% C(1) = q0^2, %C(2) = q1^2, C(3) = q2^2, C(4) = q3^2
if (C(1) > (C(2)&C(3)&C(4)))
    q_0 = sqrt(C(1));
    q_1 = (R(3,2) - R(2,3)) /(4*q_0);
    q_2 = (R(1,3) - R(3,1)) /(4*q_0);
    q_3 = (R(2,1) - R(1,2)) /(4*q_0);
end

if (C(2) > (C(1)&C(3)&C(4)))
    q_1 = sqrt(C(2));
    q_0 = (R(3,2) - R(2,3)) /(4*q_1);
    q_2 = (R(1,2) + R(2,1)) /(4*q_1);
    q_3 = (R(1,3) + R(3,1)) /(4*q_1);
end
    
if (C(3) > (C(1)&C(2)&C(4)))
    q_2 = sqrt(C(3));
    q_0 = (R(1,3) - R(3,1)) /(4*q_2);
    q_1 = (R(1,2) + R(2,1)) /(4*q_2);
    q_3 = (R(2,3) + R(3,2)) /(4*q_2);
end   
    
if (C(4) > (C(1)&C(2)&C(3)))
    q_3 = sqrt(C(4));
    q_0 = (R(2,1) - R(1,2)) /(4*q_3);
    q_1 = (R(1,3) + R(3,1)) /(4*q_3);
    q_2 = (R(2,3) + R(3,2)) /(4*q_3);
end
Q = [ q_0 q_1 q_2 q_3 ];

% Translation Quaternion
Q_t = 0.5*quatmultiply(Translation,Q);

DQ = [Q,Q_t];
end





