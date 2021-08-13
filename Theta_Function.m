function [Theta_func] = Theta_Function(DualQuat)
% Austin Henthorne MEC529 Final Project
% This function takes in the initial and final configurations as dual quaternions 
% And output the joint angles 
% STEP 3

syms c(tao)
%c(tao) = DualQuat_Interpolation([1 0 0 0 0 518.6450 100.675 0.5], [0.7071 0 0.7071 0 200 200 200 0.5]);
c(tao) = DualQuat;
DQ_interpolation = c(tao);

%************** Solving for omega_s (spatial angular velocity ************
% alpha = rotation quaternion (Real part of Dual quaternion)
alpha = DQ_interpolation(1,1:4);

alpha_0 = alpha(1,1);
alpha_1 = alpha(1,2);
alpha_2 = alpha(1,3);
alpha_3 = alpha(1,4);
J_1 = [ -alpha_1 alpha_0 -alpha_3 alpha_2;
    -alpha_2 alpha_3 alpha_0 -alpha_1;
    -alpha_3 -alpha_2 alpha_1 alpha_0];

omega_s = 2 *J_1* transpose(diff(alpha));       % 3x1


%************* Solving for v_s (spatial velocity) ******************
% p = position of end effector
% beta = Dual part of dual quaternion
beta = DQ_interpolation(1,5:8);
alpha_conj = [alpha(1,1) -alpha(1,2:4)];

% Scalar Part of beta
beta_scalar = beta(1,1);
% Vector Part of beta
beta_vector = beta(1,2:4);

% Scalar Part of alpha_conj
alpha_conj_scalar = alpha_conj(1,1);
% Vector Part of alpha_conj
alpha_conj_vector = alpha_conj(1,2:4);

quat_mult = [ (beta_scalar*alpha_conj_scalar - dot(beta_vector,alpha_conj_vector)), beta_scalar*alpha_conj_vector + alpha_conj_scalar*beta_vector + cross(beta_vector,alpha_conj_vector) ];

p = 2 * quat_mult;      % 1x4
p_star = p(1,1:3);      % 1x3
v_s_star = diff(p_star) - cross(omega_s,p_star);
v_s = [transpose(v_s_star); p(1,4)];                % 4x1


% Solving for J_2
p_hat = [ 0 -p_star(1,3) p_star(1,2);
    p_star(1,3) 0 -p_star(1,1);
    -p_star(1,2) p_star(1,1) 0];
    
J_2 = [ eye(3,3) 2*p_hat*J_1;           % 6x7
    zeros(3) 2*J_1 ];


% Solving for the spatial jacobian (J_s) = 
% J_s_star = jacobian(DQ_interpolation,tao);
% J_s_trans = transpose(J_s_star);
% J_s = [J_s_trans(1,1:7);          % 6x7

J_s = Spatial_Jacobian();   % 6x7


% ************** Temporary need to change*******
%J_s = J_2;

% 7x1 matrix of [spatial velocity; spatial angular velocity]
v_s_omegas = [v_s; omega_s];

syms B(tao)
B(tao) = transpose(J_s) * (J_s*(transpose(J_s))) * J_2;       % 7x7

syms g(tao)

g(tao) = [transpose(p_star); transpose(alpha)];         % 7x1, Transpose to get column major 


gamma_prime = [transpose(diff(p_star)); transpose(diff(alpha))];        % 7x1

% % solving for theta prime
% %Theta_prime = B * gamma_prime;
% syms gamma(tao)
% % Solving for gamma(tao)
% gamma(tao) = gamma;
% % gamma_tao = gamma;

% % Solving for gamma(tao+h)
% syms gamma_tao(tao)
% gamma_tao(tao) = gamma_tao_h;
%         


% Theta_tao = initial joint angles from initial configuration
Theta_tao = [0; 0; 0; 0; 0; 0; 0];

% Theta_tao_h = the joint angles at a certain configuration (tao)
Beta = 0.1;
h = 0.1;
n = 7;


%for tao=0:h:n*h

Theta_func = Beta*B(1)*(g(1.1) - g(1)) + Theta_tao;  % 7x1

%end


end




