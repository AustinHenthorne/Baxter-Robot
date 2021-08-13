function [Interpol] = DualQuat_Interpolation(Initial_Config, Final_Config)
% Austin Henthorne MEC529 Final Project
% This function takes in the initial and final configurations as dual quaternions 
% And outputs the interpolation equation as a function of tao
% STEP 2

% Input: DualQuat_Interpolation()
% Inital configuration = [1 0 0 0 0 518.6450 100.675 0.5]
% Final configuration = [0.7071 0 0.7071 0 200 200 200 0.5]


A = Initial_Config;
B = Final_Config;

% Real Part of A 
p = [A(1,1:4)];
% Dual Part of A
q = [A(1,5:8)];

% Scalar Part of Real of A
p_scalar = p(1,1);
% Vector Part of Real of A
p_vector = p(1,2:4);

% Scalar Part of Dual of A
q_scalar = p(1,1);
% Vector Part of Dual of A
q_vector = p(1,2:4);

% Conjugate of p
conj_p = [p(1,1) -p(1,2:4)];
% Conjugate of q
conj_q = [q(1,1) -q(1,2:4)];

% Real Part of B 
u = [B(1,1:4)];
% Dual Part of B
v = [B(1,5:8)];

% Conjugate of A
conj_A = [conj_p, -conj_q];

% Product of A_star and B
A_star_B = [quatmultiply(conj_p,u), (quatmultiply(conj_q,u) + quatmultiply(conj_p,v))]; 


% (Product of A_star and B)^ tao     (tao = interpolation parameter)

% Real Part of A_star_B 
q_1 = [A_star_B(1,1:4)];
% Dual Part of A_star_B
p_1 = [A_star_B(1,5:8)];

% Scalar part of q_1 (q_1 = rotation quaternion)
ScalarPart_q_1 = [q_1(1,1)];
% Vector part of q_1 (q_1 = rotation quaternion)
VectorPart_q_1 = [q_1(1,2:4)];

Theta = acos(ScalarPart_q_1);
% Axis of rotation (u_hat)
u_hat = VectorPart_q_1/(norm(VectorPart_q_1));  % 1x3
u_hatstar = [0 u_hat];  % 1x4

d = dot((2*quatmultiply(p_1,quatconj(q_1))),u_hatstar);

% Calculating m:
t = 2*quatmultiply(p_1,quatconj(q_1));
t_forcross = [t(1,1:3)];
cross_1 = cross(t_forcross,u_hat);
cross_star = [ 0 cross_1]; 
m = 0.5*(cross_star+(t-d*u_hatstar))*cot(Theta/2);      %1x4

New_A_star_B = (cos(Theta/2)-((d/2)*sin(Theta/2))) + ((sin(Theta/2)-((d/2)*cos(Theta/2)))*(u_hatstar+m));

% Dual Angle
Theta_bar = Theta + d;
u_hat_bar = [u_hatstar, m];


% Final Power solution (Product of A_star and B)^ tao

syms x(tao) c(tao) 
x(tao) = cos(tao*(Theta_bar/2)) + u_hat_bar*sin(tao*(Theta_bar/2));
A_star_B_tao = x(tao);


 
% syms x(tao) c(tao)
% x(tao) = (tao*cos(Theta_bar/2)) + u_hat_bar*tao*(sin(Theta_bar/2));
% A_star_B_tao = x(tao);



% Real Part of A_star_B_tao
RealPart_A_star_B_tao = [A_star_B_tao(1,1:4)];
% Dual Part of A_star_B_tao
DualPart_A_star_B_tao = [A_star_B_tao(1,5:8)];

% Scalar Part of RealPart_A_star_B_tao
Real_scalar = RealPart_A_star_B_tao(1,1);
% Vector Part of Real of A
Real_vector = RealPart_A_star_B_tao(1,2:4);

% Scalar Part of DualPart_A_star_B_tao
Dual_scalar = DualPart_A_star_B_tao(1,1);
% Vector Part of Real of A
Dual_vector = DualPart_A_star_B_tao(1,2:4);


% The below equation didnt work because of the variable tao and does not
% compute with the 'quatmultiply' function
%Interpol = [quatmultiply(p,RealPart_A_star_B_tao), (quatmultiply(q,RealPart_A_star_B_tao) + quatmultiply(p,DualPart_A_star_B_tao))];   

Interpol_Real = [ (p_scalar*Real_scalar - dot(p_vector,Real_vector)), p_scalar*Real_vector + Real_scalar*p_vector + cross(p_vector,Real_vector) ];

Interpol_Dual_1 = [ (q_scalar*Real_scalar - dot(q_vector,Real_vector)), q_scalar*Real_vector + Real_scalar*q_vector + cross(q_vector,Real_vector) ];
Interpol_Dual_2 = [ (p_scalar*Dual_scalar - dot(p_vector,Dual_vector)), p_scalar*Dual_vector + Dual_scalar*p_vector + cross(p_vector,Dual_vector) ];

Interpol_Dual = Interpol_Dual_1 + Interpol_Dual_2;

c(tao) = [Interpol_Real, Interpol_Dual];

Interpol = c(tao);


%end
end

