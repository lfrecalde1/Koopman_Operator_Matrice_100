function [A_final, B_final, G_final] = funcion_costo_koopman_lineal_csadi(X_1, X_K, U, alpha, beta, n, m, n_normal, euler)
%% Load Casadi to the path of matlab 
addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');
import casadi.*;


%% Empty vector for the optimization process
he_koop = [];
he_prediction = [];
%% Matrix Casadi Generarion
A = MX.sym('A', n, n);
B = MX.sym('B', n, m);
G = MX.sym('G', n, 1);
G_real = zeros(n, 1);
G_real(3, 1) = -3.91;

C_a = [eye(n_normal,n_normal), zeros(n_normal, n - n_normal)];

for k = 1:length(U)

    %% Minimization Koopman
    % %% Lifdted space system
    x_1 = C_a*X_1(:,k);
    x_k = C_a*X_K(:,k);

    Gamma_k = (X_K(:,k));
    Gamma_1 = (X_1(:,k));
    R = Rot_zyx(euler(:,k));
%     R = eye(3, 3);
%     R_t = [R, zeros(3, 3);...
%            zeros(3,3), R];
    
    error_koop = Gamma_k  -A*Gamma_1 - B*R*U(:,k) -G -G_real;
    
    error_prediction = x_k - C_a*(A*Gamma_1 + B*R*U(:,k) + G + G_real);
    %% Error Vector
    he_koop = [he_koop; error_koop];
    he_prediction = [he_prediction; error_prediction];
    
end
%% Optimization Problem
obj = beta*norm(he_koop, 2)^2 + alpha*norm(A, 'fro') + alpha*norm(B, 'fro') + alpha*norm(G, 'fro') + 2*norm(he_prediction, 2)^2;

%% General Vector Optimziation Variables
OPT_variables = [reshape(A,size(A,1)*size(A,2),1);reshape(B,size(B,1)*size(B,2),1);reshape(G,size(G,1)*size(G,2),1)];

nlprob = struct('f', obj, 'x', OPT_variables);
% 
opts = struct;
opts.ipopt.max_iter = 15;
% opts.ipopt.print_level =3;%0,3
opts.print_time = 1;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

%% Solver of the problem
solver = nlpsol('solver', 'ipopt', nlprob, opts);
%% Initial Conditions System
A_0 = rand(n, n)*0.1;
B_0 = rand(n, m)*0.1;
G_0 = rand(n, 1)*0.1;

args.x0 = [reshape(A_0,size(A_0,1)*size(A_0,2),1);reshape(B_0,size(B_0,1)*size(B_0,2),1);reshape(G_0,size(G_0,1)*size(G_0,2),1)]; % initial value of the optimization variables

sol = solver('x0', args.x0);
Solution = full(sol.x);

%% Full matriz system
Full_matrix = reshape(Solution, size(A,1), size(A, 2) + size(B, 2) + size(G, 2));

%% Get matrices system
A_final = Full_matrix(1:size(A,1), 1:size(A,2));
B_final = Full_matrix(1:size(B, 1), size(A,2)+1:size(A,2)+size(B,2));
G_final = Full_matrix(1:size(G, 1), size(A,2)+size(B,2)+1:size(A,2)+size(B,2)+size(G,2));

end