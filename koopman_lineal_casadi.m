%% Data Figure
%% Clear variables
clc, clear all, close all;

%% Load information
load("Data_mujoco_2.mat");
des = 1;

%% Get the data of the system
[Data_2_X_k, Data_2_X_1, Data_2_U_1, euler2] = get_data_simple_velocities(h, hp, t, T_ref);


load("Data_mujoco_1.mat");
des = 1;

%% Get the data of the system
[Data_1_X_k, Data_1_X_1, Data_1_U_1, euler1] = get_data_simple_velocities(h, hp, t, T_ref);

%% State K
X1 = [Data_1_X_1, Data_2_X_1];

%% State K+1
X2 = [Data_1_X_k, Data_2_X_k];
n_normal = size(X1,1);
%% Input K
Gamma = [Data_1_U_1, Data_2_U_1];

load("euler_estimado.mat")
euler = [euler_estimado];
%% Lifted Matrices
n = 3; 

%% Matrix values for RBF
% RBF 1 Centers
Nrbf = 2;            

cent_l = rand(n, 4)*2 - 1;   
cent_lz = rand(n, 4)*2 - 1; 
rbf_type = 'gauss';             % type of function - one of 'thinplate', 'gauss', 'invquad', 'polyharmonic'

%% Lift Space parameters
extra_param = 1;
liftFun = @(xx)( [
                 xx; 
                 rbf(xx(1,:),cent_l,rbf_type); 
                 rbf(xx(2,:), cent_l,rbf_type);
                 rbf(xx(3,:), cent_lz,rbf_type);
                 ]);
%% Lifdted space system
X1 = liftFun(X1);

%% State K+1
X2 = liftFun(X2);
 
%% Size system 
n = size(X2, 1);
m = size(Gamma, 1);

% Parameter matrices
alpha = 0.2;
beta = 0.2;
% Parametros del optimizador

%% Optimization Problem
[A_l, B_l, G_l] = funcion_costo_koopman_lineal_csadi(X1, X2, Gamma, alpha, beta, n, m, n_normal, euler);
C_l = [eye(n_normal,n_normal), zeros(n_normal, n-n_normal)];

%% Intial conditions system
v_estimate(:, 1) = C_l*(X1(:, 1));

for k= 1:length(X1)
    %% Output of the system
    salida_es(:, k) = v_estimate(:, k);
    Gamma_real = (X1(:, k));
    salida_real(:, k) = C_l*Gamma_real;
    
    %% Error of the estimation
    error(:, k) = salida_real(:,k) - salida_es(:, k);
    norm_error(k) = norm(error(:, k), 2);
    
    R = Rot_zyx(euler(:,k));
%     R_t = [R, zeros(3, 3);...
%         zeros(3,3), R];
   
    %% Evolution of the system
    v_estimate(:, k+1) = C_l*(A_l*liftFun(v_estimate(:, k)) + B_l*R*Gamma(:,k) + G_l);
end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(salida_real(1,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(salida_es(1,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'${{v_x}}$','$\hat{v_x}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angles estimation}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(salida_real(2,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(salida_es(2,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${v_y}$','$\hat{v_y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(salida_real(3,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(salida_es(3,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${v_z}$','$\hat{v_z}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

set(gcf, 'Color', 'w'); % Sets axes background
export_fig velocities_estimation_koopman.pdf -q101



figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(1,1,1)
plot(norm_error(1,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'$||e_{estimation}||$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Error estimation}$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background
export_fig norm_estimation_velocities_koopman.pdf -q101
save("matrices_lineal.mat", "A_l", "B_l", "G_l", "C_l", "cent_l", "cent_lz")
figure
imagesc(A_l);

figure
imagesc(B_l);

figure
imagesc(G_l);