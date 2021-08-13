function [Js] = Spatial_Jacobian()

% Austin Henthorne MEC529 Final Project
% This function calculates the spatial manipulator jacobian
% STEP 3: Part 1 of 2


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


% Calculating the -wxq (all revolute joints); first part of xi
cross1 = cross(-w1r,q1r);
cross2 = cross(-w2r,q2r);
cross3 = cross(-w3r,q3r);
cross4 = cross(-w4r,q4r);
cross5 = cross(-w5r,q5r);
cross6 = cross(-w6r,q6r);
cross7 = cross(-w7r,q7r);

% calculating the xis
xi1 = [cross1; w1r];
xi2 = [cross2; w2r];
xi3 = [cross3; w3r];
xi4 = [cross4; w4r];
xi5 = [cross5; w5r];
xi6 = [cross6; w6r];
xi7 = [cross7; w7r];

% solving for R's matrices for Adg
what_1 = [ 0 -w1r(3,1) w1r(2,1);
    w1r(3,1) 0 -w1r(1,1);
    -w1r(2,1) w1r(1,1) 0];
what_2 = [ 0 -w2r(3,1) w2r(2,1);
    w2r(3,1) 0 -w2r(1,1);
    -w2r(2,1) w2r(1,1) 0];
what_3 = [ 0 -w3r(3,1) w3r(2,1);
    w3r(3,1) 0 -w3r(1,1);
    -w3r(2,1) w3r(1,1) 0];
what_4 = [ 0 -w4r(3,1) w4r(2,1);
    w4r(3,1) 0 -w4r(1,1);
    -w4r(2,1) w4r(1,1) 0];
what_5 = [ 0 -w5r(3,1) w5r(2,1);
    w5r(3,1) 0 -w5r(1,1);
    -w5r(2,1) w5r(1,1) 0];
what_6 = [ 0 -w6r(3,1) w6r(2,1);
    w6r(3,1) 0 -w6r(1,1);
    -w6r(2,1) w6r(1,1) 0];
what_7 = [ 0 -w7r(3,1) w7r(2,1);
    w7r(3,1) 0 -w7r(1,1);
    -w7r(2,1) w7r(1,1) 0];

% calculating the q_hat matrices for the Adg
qhat_1 = [ 0 -q1r(3,1) q1r(2,1);
    q1r(3,1) 0 -q1r(1,1);
    -q1r(2,1) q1r(1,1) 0];
qhat_2 = [ 0 -q2r(3,1) q2r(2,1);
    q2r(3,1) 0 -q2r(1,1);
    -q2r(2,1) q2r(1,1) 0];
qhat_3 = [ 0 -q3r(3,1) q3r(2,1);
    q3r(3,1) 0 -q3r(1,1);
    -q3r(2,1) q3r(1,1) 0];
qhat_4 = [ 0 -q4r(3,1) q4r(2,1);
    q4r(3,1) 0 -q4r(1,1);
    -q4r(2,1) q4r(1,1) 0];
qhat_5 = [ 0 -q5r(3,1) q5r(2,1);
    q5r(3,1) 0 -q5r(1,1);
    -q5r(2,1) q5r(1,1) 0];
qhat_6 = [ 0 -q6r(3,1) q6r(2,1);
    q6r(3,1) 0 -q6r(1,1);
    -q6r(2,1) q6r(1,1) 0];
qhat_7 = [ 0 -q7r(3,1) q7r(2,1);
    q7r(3,1) 0 -q7r(1,1);
    -q7r(2,1) q7r(1,1) 0];

% Calculate Adg's
Adg1 = [what_1 (qhat_1*what_1);
    zeros(3,3) what_1];
Adg2 = [what_2 (qhat_2*what_2);
    zeros(3,3) what_2];
Adg3 = [what_3 (qhat_3*what_3);
    zeros(3,3) what_3];
Adg4 = [what_4 (qhat_4*what_4);
    zeros(3,3) what_4];
Adg5 = [what_5 (qhat_5*what_5);
    zeros(3,3) what_5];
Adg6 = [what_6 (qhat_6*what_6);
    zeros(3,3) what_6];
Adg7 = [what_7 (qhat_7*what_7);
    zeros(3,3) what_7];

% Calaculate xi primes
xi2_prime = Adg2*xi2;
xi3_prime = Adg3*xi3;
xi4_prime = Adg4*xi4;
xi5_prime = Adg5*xi5;
xi6_prime = Adg6*xi6;
xi7_prime = Adg7*xi7;

Js = [xi1 xi2_prime xi3_prime xi4_prime xi5_prime xi6_prime xi7_prime];

