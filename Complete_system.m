%% Simulation of the entire system
clc, clear all, close all;


%% Load Data of the system
load("matrices_lineal.mat");
load("matrices_angular.mat");
%% Load information
load("h_2.mat");
load("hp_2.mat");
load("T_ref_2.mat");
load("t_2.mat");
des = 1;

T_ref(1, :) = T_ref(1, :)/100;
%% Time Definition
ts = t(end)-t(end-1);

%% Load Data System Pose
h = h(:, des:end-1);

%% Load Data Velocities
hp = hp(:, des:end-1);

%% Load Time
t = t(:,des:end);

%% Length Simulation
N = length(t);


%% Real velocities Body
ul_w = hp(1, :);
um_w = hp(2, :);
un_w = hp(3, :);



%% General Vector Velocities Body
u_w = [ul_w; um_w; un_w];

%% Real Angles System
phi = h(8, :);
theta = h(9,:);
psi = h(10, :);

euler = [phi;...
         theta;...
         psi];
%% Get angular 
for k =1:length(hp)
[euler_p(:, k)] = Euler_p(hp(4:6, k),h(8:10, k));
end

%% Angular Velocities Body euler
phi_p = euler_p(1, :);
theta_p = euler_p(2, :);
psi_p = euler_p(3, :);     

angular_states = [euler; euler_p];
%% Gaussian function definition
Nrbf = 2;            
n_l = 3;

rbf_type_linel = 'gauss';             % type of function - one of 'thinplate', 'gauss', 'invquad', 'polyharmonic'

%% Lift Space parameters
extra_param = 1;
liftFun_lineal = @(xx)( [
                 xx; 
                 rbf(xx(1,:),cent_l,rbf_type_linel); 
                 rbf(xx(2,:), cent_l,rbf_type_linel);
                 rbf(xx(3,:), cent_lz,rbf_type_linel);
                 ]);
%% Lifted Matrices
n_a = 6; 

%% Matrix values for RBF

% RBF 1 Centers
Nrbf = 2;            

  
rbf_type_angular = 'gauss';             % type of function - one of 'thinplate', 'gauss', 'invquad', 'polyharmonic'

liftFun_angular = @(xx)( [
                 xx; 
                 rbf(xx(4,:),cent_a,rbf_type_angular); 
                 rbf(xx(5,:), cent_a,rbf_type_angular);
                 rbf(xx(6,:), cent_a,rbf_type_angular);
                 ]);
                         
%% Initial linear velocities
v_linear_estimate(:, 1) = liftFun_lineal(u_w(:, 1));
y_linear_estimate(:, 1) = C_l*v_linear_estimate(:, 1);

%% Initial angular velocities
v_angular_estimate(:, 1) = liftFun_angular(angular_states(:, 1));
y_angular_estimate(:, 1) = C_a*v_angular_estimate(:, 1);
tic
for k= 1:length(t)
    
    %% Output of the system lineal
    Gamma_real_lineal = liftFun_lineal(u_w(:, k));
    salida_real_lineal(:, k) = C_l*Gamma_real_lineal;
    
    %% Error of the estimation
    error_l(:, k) = salida_real_lineal(:,k) - y_linear_estimate(:, k);
    norm_error_l(k) = norm(error_l(:, k), 2);
    
    %% Output of the system angular
    Gamma_real_angular = liftFun_angular(angular_states(:, k));
    salida_real_angular(:, k) = C_a*Gamma_real_angular;
    
    %% Error of the estimation
    error_a(:, k) = salida_real_angular(:,k) - y_angular_estimate(:, k);
    norm_error_a(k) = norm(error_a(:, k), 2);
    
    R = Rot_zyx(y_angular_estimate(:, k));
   
    %% Evolution of the system
    v_angular_estimate(:, k+1) = A_a*v_angular_estimate(:, k) + B_a*T_ref(2:4, k);
    y_angular_estimate(:, k+1) = C_a*v_angular_estimate(:, k);
    
    
    
    v_linear_estimate(:, k+1) = A_l*v_linear_estimate(:, k) + B_l*R*[0;0;T_ref(1,k)] + G_l;
    y_linear_estimate(:, k+1) = C_l*v_linear_estimate(:, k);

end
toc
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(salida_real_lineal(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(y_linear_estimate(1,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'${{v_x}}$','$\hat{v_x}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Velocities estimation}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(salida_real_lineal(2,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(y_linear_estimate(2,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${v_y}$','$\hat{v_y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(salida_real_lineal(3,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(y_linear_estimate(3,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${v_z}$','$\hat{v_z}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);


figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(1,1,1)
plot(norm_error_l(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'$||e_{estimation}||$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Error estimation}$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(salida_real_angular(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(y_angular_estimate(1,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'${{\phi}}$','$\hat{\phi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angles estimation}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(salida_real_angular(2,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(y_angular_estimate(2,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\theta}$','$\hat{\theta}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(salida_real_angular(3,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(y_angular_estimate(3,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\psi}$','$\hat{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);


figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(salida_real_angular(4,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(y_angular_estimate(4,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'$\dot{{\phi}}$','$\dot{\hat{\phi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angles estimation}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(salida_real_angular(5,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(y_angular_estimate(5,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'$\dot{{\theta}}$','$\dot{\hat{\theta}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(salida_real_angular(6,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(y_angular_estimate(6,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\dot{\psi}}$','$\dot{\hat{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(1,1,1)
plot(norm_error_a(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'$||e_{estimation}||$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Error estimation}$','Interpreter','latex','FontSize',9);


figure
imagesc(A_a);

figure
imagesc(B_a);

figure
imagesc(A_l);

figure
imagesc(B_l);

figure
imagesc(G_l);
