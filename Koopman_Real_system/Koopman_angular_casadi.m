%% Koopman identification using casadi as a optimization framework
clc, clear all, close all;


%% Load experimentatl information
% load("h_2.mat");
% load("hp_2.mat");
% load("T_ref_2.mat");
% load("t_2.mat");
% load("Data_mujoco_2.mat");
% %% geta Matrices of the system
% [Data_2_X_k, Data_2_X_1, Data_2_U_1] = get_data_simple(h, hp, u, t, T_ref);

% load("h_3.mat");
% load("hp_3.mat");
% load("T_ref_3.mat");
% load("t_3.mat");
load("Data_DJI_1.mat");
[Data_1_X_k, Data_1_X_1, Data_1_U_1] = get_data_simple(h, hp, u, t, T_ref);

%% Rearrange data in order to develp DMD ext
%% State K
X1 = [Data_1_X_1];

%% State K+1
X2 = [Data_1_X_k];
n_normal = size(X1,1);
%% Input K
Gamma = [Data_1_U_1];

%% Lifted Matrices
n_a = 3; 

%% Matrix values for RBF aproximation
Nrbf = 2;            

cent_a = rand(n_a,Nrbf)*2 - 1;   
rbf_type = 'gauss';             % type of function - one of 'thinplate', 'gauss', 'invquad', 'polyharmonic'

extra_param = 1;
liftFun = @(xx)( [
                 xx;
                 sin(xx(1, :)).*tan(xx(2, :));...
                 cos(xx(1, :)).*tan(xx(2, :));...
                 cos(xx(1, :));...
                 sin(xx(1, :));...
                 sin(xx(1,:))./cos(xx(2, :));...
                 cos(xx(1,:))./cos(xx(2, :));...
                 xx(7, :).*xx(8, :);...
                 xx(7, :).*xx(9, :);...
                 xx(8, :).*xx(9, :)]);
             

%% Lifdted space system
X1 = liftFun(X1);

%% State K+1
X2 = liftFun(X2);

%% Size system 
n = size(X2, 1);
m = size(Gamma, 1);
%% Optimization  variables 
alpha = 0.2;
beta = 0.2;

%% Optimization Problem
[A_a, B_a, G_a] = funcion_costo_koopman_csadi(X1, X2, Gamma, alpha, beta, n, m, n_normal);
C_a = [eye(n_normal,n_normal), zeros(n_normal, n-n_normal)];

%%
v_estimate(:, 1) = C_a*(X1(:, 1));
for k= 1:length(X1)
    %% Output of the system
    salida_es(:, k) = v_estimate(:, k);
    Gamma_real = (X1(:, k));
    salida_real(:, k) = C_a*Gamma_real;
    
    %% Error of the estimation
    error(:, k) = salida_real(:,k) - salida_es(:, k);
    norm_error(k) = norm(error(:, k), 2);
    
    
    %% Evolution of the system
    v_estimate(:, k+1) = C_a*(A_a*liftFun(v_estimate(:, k)) + B_a*Gamma(:, k) + G_a);
    
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
legend({'${{\phi}}$','$\hat{\phi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angles estimation}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(salida_real(2,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(salida_es(2,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\theta}$','$\hat{\theta}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(salida_real(3,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(salida_es(3,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\psi}$','$\hat{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

set(gcf, 'Color', 'w'); % Sets axes background
export_fig angles_estimation_koopman.pdf -q101

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(salida_real(4,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(v_estimate(4,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'$\dot{{\phi}}$','$\dot{\hat{\phi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angular velocity estimation}$','Interpreter','latex','FontSize',9);
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(salida_real(5,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(v_estimate(5,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'$\dot{\theta}$','$\dot{\hat{\theta}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(salida_real(6,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(v_estimate(6,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\dot{\psi}}$','$\dot{\hat{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);

set(gcf, 'Color', 'w'); % Sets axes background
export_fig euler_p_estimation_koopman.pdf -q101

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(salida_real(10,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(v_estimate(10,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'$v_x$','$\hat{v_x}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angular velocity estimation}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(salida_real(11,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(v_estimate(11,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'$v_y$','$\hat{v_y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(salida_real(12,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(v_estimate(12,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
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
export_fig norm_angles_estimation_koopman.pdf -q101

save("matrices_angular.mat", "A_a", "B_a", "C_a", "cent_a")
euler_estimado = v_estimate;
save("euler_estimado.mat", "euler_estimado");
figure
imagesc(A_a);

figure
imagesc(B_a);

figure
imagesc(G_a);
