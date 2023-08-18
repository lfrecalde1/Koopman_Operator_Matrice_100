%% Data Figure
%% Clear variables
clc, clear all, close all;

%% Load information
load("h_2.mat");
load("hp_2.mat");
load("T_ref_2.mat");
load("t_2.mat");
des = 1;

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


%% Filter Parameters
landa = 10;%lambda
F1=tf(landa,[1 landa]);

%% Real velocities Body
ul_w = hp(1, :);
um_w = hp(2, :);
un_w = hp(3, :);


%% Filter Body Velocities
ul_w_f=lsim(F1,ul_w,t)';
um_w_f=lsim(F1,um_w,t)';
un_w_f=lsim(F1,un_w,t)';

%% General Vector Velocities Body
u_w = [ul_w; um_w; un_w];
u_w_f = [ul_w_f; um_w_f; un_w_f];
%% Get Aceleration System Body
for k=1:length(t)
    if k>1
        ul_w_p(k)=(ul_w(k)- ul_w(k-1))/ts;
        um_w_p(k)=(um_w(k)- um_w(k-1))/ts;
        un_w_p(k)=(un_w(k)- un_w(k-1))/ts;     
    else
        ul_w_p(k)=0;
        um_w_p(k)=0;
        un_w_p(k)=0;
        
    end
end

%% Filter Aceleration Body
ul_w_p_f=lsim(F1,ul_w_p,t)';
um_w_p_f=lsim(F1,um_w_p,t)';
un_w_p_f=lsim(F1,un_w_p,t)';

%% General Vector Aceleration Body
u_w_p = [ul_w_p; um_w_p; un_w_p];
up_w_f = [ul_w_p_f; um_w_p_f; un_w_p_f];

%% Real Angles System
phi = h(8, :);
theta = h(9,:);
psi = h(10, :);


euler = h(8:10, :);   


%% Optimizer parameters
options = optimset('Display','iter',...
    'TolFun', 1e-8,...
    'MaxIter', 60000,...
    'Algorithm', 'active-set',...
    'FinDiffType', 'forward',...
    'RelLineSrchBnd', [],...
    'RelLineSrchBndDuration', 1,...
    'TolConSQP', 2e-8);

%% Initial Condition Optimization problem
x0=ones(1,10);
f_obj1 = @(x) cost_function(x, N, T_ref, u_w, u_w_p, euler);
%% Optimization Problem
x = fmincon(f_obj1,x0,[],[],[],[],[],[],[],options);
chi = x

v_estimate(:, 1) = u_w(:,1);

for k = 1:length(t)-1
    
    v_estimate(:, k+1) = aerial_system_dynamics(v_estimate(:, k), h(:, k), T_ref(1, k), ts, chi);
end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

subplot(3,1,1)
plot(u_w(1,1:length(v_estimate))','-','Color',[105, 123, 216]/255,'linewidth',1); hold on
grid on;
plot(v_estimate(1,:)','--','Color',[105, 123, 216]/255,'linewidth',1); hold on
legend({'${v_x}$','$\hat{v}_x$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);


subplot(3,1,2)
plot(u_w(2,1:length(v_estimate))','-','Color',[105, 123, 216]/255,'linewidth',1); hold on
grid on;
plot(v_estimate(2,:)','--','Color',[105, 123, 216]/255,'linewidth',1); hold on
legend({'${v_y}$','$\hat{v}_y$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);


subplot(3,1,3)
plot(u_w(3,1:length(v_estimate))','-','Color',[105, 123, 216]/255,'linewidth',1); hold on
grid on;
plot(v_estimate(3,:)','--','Color',[105, 123, 216]/255,'linewidth',1); hold on
legend({'${v_z}$','$\hat{v}_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
