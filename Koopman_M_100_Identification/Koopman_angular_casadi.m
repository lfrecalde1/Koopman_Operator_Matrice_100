%% Koopman identification using casadi as a optimization framework
clc, clear all, close all;


%% Load information
load("h_6.mat");
load("hp_6.mat");
load("hdp_6.mat");
load("rdp_6.mat");
load("t_6.mat");
load("u_ref_6.mat");

%% Get Body Velocities 
for k = 1:length(h)
   u(:,k) =  inv(Rot_zyx(h(8:10, k)))*[hp(1, k); hp(2, k); hp(3, k)];  
end

for k =1:length(t)
[euler_p(:, k)] = Euler_p(hp(4:6, k),h(8:10, k));
end

%% Split Velocity
ul = u(1, :);
um = u(2, :);
un = u(3, :);
p = hp(4, :);
q = hp(5, :);
r = hp(6, :);

%% Split Forces and Torques
T_ref = u_ref;

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
                 sin(xx(1, :)).*tan(xx(2, :)).*xx(5, :);...
                 cos(xx(1, :)).*tan(xx(2, :)).*xx(6, :);...
                 cos(xx(1, :)).*xx(5, :);...
                 sin(xx(1, :)).*xx(6, :);...
                 sin(xx(1,:))./cos(xx(2, :)).*xx(5, :);...
                 cos(xx(1,:))./cos(xx(2, :)).*xx(6, :);...
                 xx(4, :).*xx(5, :);...
                 xx(4, :).*xx(6, :);...
                 xx(5, :).*xx(6, :)]);
             

%% Lifdted space system
X1 = liftFun(X1);

%% State K+1
X2 = liftFun(X2);

%% Size system 
n = size(X2, 1);
m = size(Gamma, 1);
%% Optimization  variables 
alpha = 0.01;
beta = 0.01;

%% Optimization Problem
[A_a, B_a] = funcion_costo_koopman_csadi(X1, X2, Gamma, alpha, beta, n, m, n_normal);
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
    v_estimate(:, k+1) = C_a*(A_a*liftFun(v_estimate(:, k)) + B_a*Gamma(:, k));
    
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


save("matrices_angular.mat", "A_a", "B_a", "C_a", "cent_a")
euler_estimado = v_estimate;
save("euler_estimado.mat", "euler_estimado");
figure
imagesc(A_a);

figure
imagesc(B_a);
