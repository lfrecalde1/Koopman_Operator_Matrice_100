%% Syten Data Adquisition Mujoco

%% Clear Variables
clc, clear all, close all;

%% Load Data of the system
load("matrices_lineal.mat");
load("matrices_angular.mat");

%% Definition Sample Time
ts = 0.03;
tfinal = 10;
t = (0:ts:tfinal);

%% Functions Definition
rbf_type_linel = 'gauss';             % type of function - one of 'thinplate', 'gauss', 'invquad', 'polyharmonic'

%% Lift Space parameters
extra_param = 1;
liftFun_lineal = @(xx)( [
                 xx; 
                 rbf(xx(1,:),cent_l,rbf_type_linel); 
                 rbf(xx(2,:), cent_l,rbf_type_linel);
                 rbf(xx(3,:), cent_lz,rbf_type_linel);
                 ]);

%% Matrix values for RBF
liftFun_angular = @(xx)( [
                 xx;
                 sin(xx(1, :)).*tan(xx(2, :));...
                 cos(xx(1, :)).*tan(xx(2, :));...
                 cos(xx(1, :));...
                 sin(xx(1, :));...
                 sin(xx(1,:))./cos(xx(2, :));...
                 cos(xx(1,:))./cos(xx(2, :));...
                 xx(4, :).*xx(5, :);...
                 xx(4, :).*xx(6, :);...
                 xx(5, :).*xx(6, :);...
                 cos(xx(1, :)).*sin(xx(2, :)).*cos(xx(3, :)) + sin(xx(1, :)).* sin(xx(3, :));...
                 cos(xx(1, :)).*sin(xx(2, :)).*sin(xx(3, :)) - sin(xx(1, :)).* cos(xx(3, :));...
                 cos(xx(1, :)).*cos(xx(2, :))]);
             
liftFun = @(xx)( [
                 liftFun_lineal(xx(1:3,:)); 
                 liftFun_angular(xx(4:9,:));
                 ]);
             
%% Matrices of the system 
A = [A_l, zeros(size(A_l,1),size(A_a,2));...
     zeros(size(A_a,1),size(A_l,2)), A_a];
 
B = [B_l, zeros(size(B_l,1),size(B_a,2));...
     zeros(size(B_a,1),size(B_l,2)), B_a];
 
C = [C_l, zeros(size(C_l,1),size(C_a,2));...
     zeros(size(C_a,1),size(C_l,2)), C_a];

G = [G_l;zeros(size(A_a, 1),1)];

%% Ros Configuration
rosshutdown
rosinit('192.168.1.106', 'NodeHost', '192.168.1.106', 'Nodename', '/Matlab');

%% Ros topics names
robot_references = rospublisher('/cmd_vel');
velmsg = rosmessage(robot_references);
odom = rossubscriber('/odom');
inputs = rossubscriber('/input_ref');
send_velocities(robot_references, velmsg, [0, 0, 0, 0, 0 , 0])

%% Initial Conditions System
h = zeros(10, length(t)+1);
hp = zeros(6, length(t)+1);
u = zeros(3, length(t)+1);
T = zeros(3, length(t));
F = zeros(3, length(t));
omega_ref = zeros(2, length(t));

%% Get data system
[h(:, 1), hp(:, 1), u(:, 1)] = odometry(odom);

%% Initial linear velocities
v_linear_estimate(:, 1) = hp(1:3, 1);

%% Initial angular velocities
v_angular_estimate(:, 1) = [h(8:10, 1); hp(4:6,1)];

v_estimate(:, 1) = [v_linear_estimate(:,1);v_angular_estimate(:,1)];

%% Desired Estates
x_desired = [0.1*ones(1, length(t));...
             0.1*ones(1, length(t));...
             0.1*ones(1, length(t));...
             0*ones(1, length(t));...
             0*ones(1, length(t));...
             0*ones(1, length(t));...
             0*ones(1, length(t));...
             0*ones(1, length(t));...
             0*ones(1, length(t))];
      
%% Desired and real lift space
x_desire_lift = liftFun(x_desired);
xlift(:, 1) = liftFun(v_estimate(:, 1));

%% Setup controller
bounded = [15; 2; 0.2; -0.2; 0.2; -0.2; 1.5; -1.5];

%% OPTIMIZATION SOLVER
N_mpc = 5;

%% Solver setup
[f,solver,args] = mpc_drone(bounded, N_mpc, A, B, G, ts);

%% Initial COnditions controller 
vc = zeros(N_mpc,4);

H0 = repmat(xlift(:, 1),1,N_mpc+1)';

%% Control actions of the system
Control_signals = zeros(4, length(t)-N_mpc);
%% Init System
for i = 1:200
    tic;
    send_full(robot_references, velmsg, [0, 0, 0, 0, 0, 0]);
    [aux_h, aux_hp, aux_u] = odometry(odom);
    while(toc<ts)
    end
end

for k=1:1:length(t)-N_mpc
    tic; 
    tic
    [H0, control] = NMPC(liftFun(v_estimate(:, k)) , x_desire_lift(:, :), k, H0, vc, args, solver ,N_mpc);
    
    Control_signals(1, k) = control(1,1);
    Control_signals(2, k) = control(1,2);
    Control_signals(3, k) = control(1,3);
    Control_signals(4, k) = control(1,4);

    %% Control Section
    %% Send control values to the robot
    send_full(robot_references, velmsg, [0, 0, Control_signals(1, k), Control_signals(2, k), Control_signals(3, k), Control_signals(4, k)]);
    
    
    while(toc<ts)
        [F(:, k), T(:, k)] =  inputs_system(inputs);
        [omega_ref(:, k)] = inputs_system_rate(inputs);
    end
    %% Getas data from the drone
    [h(:, k+1), hp(:,k+1), u(:, k+1)] = odometry(odom);
    %% Initial linear velocities
    v_linear_estimate(:, k+1) = hp(1:3, k+1);
    
    %% Initial angular velocities
    v_angular_estimate(:, k+1) = [h(8:10, k+1); hp(4:6, k+1)];
    
    v_estimate(:, k+1) = [v_linear_estimate(:, k+1);v_angular_estimate(:, k+1)];
    t_real(k) = toc;
    %% update control and states values
    vc = [control(2:end,:);control(end,:)];
    H0 = [H0(2:end,:);H0(end,:)];
    toc
end
send_velocities(robot_references, velmsg, [0, 0, 0, 0, 0, 0])
rosshutdown;

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(4,1,1)
plot(t(1, 1:length(v_estimate)), v_estimate(1,:),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
plot(t(1, 1:length(v_estimate)),x_desired(1,1:length(v_estimate)),'-','Color',[50,76,10]/255,'linewidth',1); hold on
legend({'$\hat{v_x}$', '${v_x}_d$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Velocities estimation}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

subplot(4,1,2)
plot(t(1, 1:length(v_estimate)),v_estimate(2,:),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
plot(t(1, 1:length(v_estimate)),x_desired(2,1:length(v_estimate)),'-','Color',[50,76,10]/255,'linewidth',1); hold on
legend({'$\hat{v_y}$', '${v_y}_d$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(4,1,3)
plot(t(1, 1:length(v_estimate)),v_estimate(3,:),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
plot(t(1, 1:length(v_estimate)),x_desired(3,1:length(v_estimate)),'-','Color',[50,76,10]/255,'linewidth',1); hold on
legend({'$\hat{v_z}$', '${v_z}_d$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);


subplot(4,1,4)
plot(t(1, 1:length(v_estimate)),v_estimate(9,:),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
plot(t(1, 1:length(v_estimate)),x_desired(9,1:length(v_estimate)),'-','Color',[50,76,10]/255,'linewidth',1); hold on
legend({'$\hat{\dot{\psi}}$', '$\dot{\psi}_d$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

