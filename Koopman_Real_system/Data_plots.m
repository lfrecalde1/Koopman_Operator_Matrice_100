%% Data Figure
%% Clear variables
clc, clear all, close all;

%% Load information
load("h_6.mat");
load("hp_6.mat");
load("hdp_6.mat");
load("rdp_6.mat");
load("t_6.mat");
load("u_ref_6.mat");

for k = 1:length(h)
   u(:,k) =  inv(Rot_zyx(h(8:10, k)))*[hp(1, k); hp(2, k); hp(3, k)];  
end

for k =1:length(t)
[euler_p(:, k)] = Euler_p(hp(4:6, k),h(8:10, k));
end


% %% Split Velocity
ul = u(1, :);
um = u(2, :);
un = u(3, :);

p = hp(4, :);
q = hp(5, :);
r = hp(6, :);
% 
% %% Split Forces and Torques
ul_ref = hdp(1, :);
um_ref = hdp(2, :);
un_ref = hdp(3, :);

w_ref = rdp(3, :);

%% Control Action
fz = u_ref(1, :);
wx_ref = u_ref(2, :);
wy_ref = u_ref(3, :);
tz = u_ref(4, :);

T_ref = [u_ref(1,:);...
         u_ref(2,:);...
         u_ref(3, :);...
         u_ref(4,:)];
save("Data_DJI_3.mat", "t", "T_ref", "h", "hp", "hdp", "u", "ul_ref", "um_ref", "un_ref", "w_ref")

% %% Images System
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
plot(t(1:length(ul_ref)),hp(3, 1:length(t)),'--','Color',[26,115,160]/255,'linewidth',1); hold on
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
