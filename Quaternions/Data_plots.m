%% Data Figure
%% Clear variables
clc, clear all, close all;

%% Load information
load("Data_mujoco_1.mat");

%% Split Velocity
ul = u(1, :);
um = u(2, :);
un = u(3, :);
p = hp(4, :);
q = hp(5, :);
r = hp(6, :);

%% Split Forces and Torques

fz = T_ref(1, :);


tz = T_ref(4, :);

wx_ref = T_ref(2, :);
wy_ref = T_ref(3, :);

%% Angles velocities
for k =1:length(t)
[euler_p(:, k)] = Euler_p(hp(4:6, k),h(8:10, k));
end

%% Images System
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(4,1,1)
plot(t(1:length(ul_ref)),ul_ref,'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul_ref)),ul(1,1:length(t)),'--','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'$\mu_{lc}$','$\mu_{l}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(4,1,2)
plot(t(1:length(ul_ref)),um_ref,'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(ul_ref)),um(1,1:length(t)),'--','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'$\mu_{mc}$','$\mu_{m}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(4,1,3)
plot(t(1:length(ul_ref)),un_ref,'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(ul_ref)),un(1:length(t)),'--','Color',[26,115,160]/255,'linewidth',1); hold on
grid on;
legend({'$\mu_{nc}$','$\mu_{n}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(4,1,4)
plot(t(1:length(ul_ref)),w_ref,'Color',[83,57,217]/255,'linewidth',1); hold on
plot(t(1:length(ul_ref)),r(1,1:length(t)),'--','Color',[83,57,217]/255,'linewidth',1); hold on
grid on;
legend({'$\omega_{c}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Velocities Body.pdf -q101
% 
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);


subplot(4,1,3)
plot(t(1:length(ul_ref)),fz(1:length(t)),'-','Color',[26,115,160]/255,'linewidth',1); hold on
grid on;
legend({'$f_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[N]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(4,1,4)
plot(t(1:length(ul_ref)),tz(1,1:length(t)),'-','Color',[83,57,217]/255,'linewidth',1); hold on
grid on;
legend({'$\tau_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Forces_and_torque.pdf -q101

% 
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(4,1,1)
plot(t(1:length(ul_ref)),q(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'$q$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(4,1,2)
plot(t(1:length(ul_ref)),p(1,1:length(t)),'-','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'$p$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(4,1,3)
plot(t(1:length(ul_ref)),un(1:length(t)),'-','Color',[26,115,160]/255,'linewidth',1); hold on
grid on;
legend({'$\mu_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(4,1,4)
plot(t(1:length(ul_ref)),r(1,1:length(t)),'-','Color',[83,57,217]/255,'linewidth',1); hold on
grid on;
legend({'$r$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Rates_and_velocity.pdf -q101



figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(4,1,1)
plot(t(1:length(ul_ref)),wy_ref(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(ul_ref)),h(9,1:length(t)),'--','Color',[226,76,44]/255,'linewidth',1); hold on
legend({'$\theta_d$','$\theta$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(4,1,2)
plot(t(1:length(ul_ref)),wx_ref(1,1:length(t)),'-','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(ul_ref)),h(8,1:length(t)),'--','Color',[46,188,89]/255,'linewidth',1); hold on
legend({'$\phi_d$','$\phi$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(4,1,3)
plot(t(1:length(ul_ref)),fz(1:length(t)),'-','Color',[26,115,160]/255,'linewidth',1); hold on
grid on;
legend({'$f_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[N]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(4,1,4)
plot(t(1:length(ul_ref)),tz(1,1:length(t)),'-','Color',[83,57,217]/255,'linewidth',1); hold on
grid on;
legend({'$\tau_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Forces_and_torque_angles.pdf -q101



figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
plot(t(1:length(ul_ref)),euler_p(2,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(ul_ref)),h(9,1:length(t)),'--','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul_ref)),q(1,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'$\dot{\theta}$','$\theta$','$q$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(2,1,2)
plot(t(1:length(ul_ref)),euler_p(1,1:length(t)),'-','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(ul_ref)),h(8,1:length(t)),'--','Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(ul_ref)),p(1,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'$\dot{\phi}$','$\phi$','$p$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

for k = 1:length(ul_ref)
   vd_world(:,k) =  Rot_zyx(h(8:10, k))*[ul_ref(1, k); um_ref(1, k); un_ref(1, k)];  
end

save("reference_world.mat", "vd_world")
