%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXX TRAJECTORY CONTROL DJI DRONE XXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

%% CLEAN VARIABLES
clc,clear all,close all;

%% DEFINITION OF TIME VARIABLES
ts = 0.1;
tf = 100;
to = 0;
t = (to:ts:tf);

%% CONSTANTS VALUES OF THE ROBOT
a = 0.1; 
b = 0.1;
c = 0.0;
r = 0.45*ones(1,length(t));
L = [a, b, c, r(1)];

%% INITIAL CONDITIONS
x = 0.0;
y = 0.0;
z = 0;
yaw = 0*(pi/180);

%% DIRECT KINEMATICS
x = x +a*cos(yaw) - b*sin(yaw);
y = y +a*sin(yaw) + b*cos(yaw);
z = z + c;

h = [x;...
     y;...
     z;...
     yaw];
 
%% INITIAL GENERALIZE VELOCITIES
v = [0;...
     0;...
     0;...
     0];

H = [h;v];
%% DESIRED SIGNALS OF THE SYSYEM
%% DESIRED SIGNALS OF THE SYSYEM
[hxd, hyd, hzd, hthd] = Trajectory(t,ts,4);

%% GENERALIZED DESIRED SIGNALS
hd = [hxd;...
      hyd;...
      hzd;...
      hthd];
  
object1 = [-0.6;...
            1.62;...
            7.4]; 
        
object2 = [6.8;...
            3.3;...
            7.2];
        
object3 = [-8.9;...
            2.1;...
            6.99];        
        
ob = [object1(:,1),object2(:,1),object3(:,1)];
%% LOAD DYAMIC PARAMETERS DRONE
load("parameters.mat");


%% Definicion del horizonte de prediccion
N = 9; 

%% Definicion de los limites de las acciondes de control
bounded = [1.2; -1.2; 1.2; -1.2; 1.2; -1.2; 1.5; -1.5];
%% Definicion del vectro de control inicial del sistema
vc = zeros(N,4);
H0 = repmat(H,1,N+1)';

%% OPTIMIZATION SOLVER
 [f, solver, args] = mpc_drone(bounded, N, L, chi, ob, ts);
%  [f, solver, args] = mpc_drone_cinematica(bounded, N, L, ts);

%% DISTANCE TO OBJECTS
[d] = distance_objects(ob, h);
%% SIMULATION 
for k=1:1:length(t)-N
    tic; 
    %% GENERAL VECTOR OF ERROR SYSTEM
    he(:, k) = hd(:,k)-h(:,k);
    
    %% OPTIMAL CONTROLLER SECTION
    %% STATES OF THE OPTIMAL CONTROLLER
    [H0, control] = NMPC(h(:,k), v(:,k), hd(:,:), k, H0, vc, args, solver, N, ob);
%     [H0, control] = NMPC_cinematica(h(:,k), hd(:,:), k, H0, vc, args, solver, N);
    
    %% OBTAIN CONTROL VALUES OF THE VECTOR
    ul(k) = control(1,1);
    um(k) = control(1,2);
    un(k) = control(1,3);
    w(k) = control(1,4);
    
    %% GET VALUES OF DRONE
    
    v(:,k+1) = system_dynamic(chi, v(:,k), control(1,:)', ts);
    [h(:,k+1), hp(:,k+1)] = system_drone(h(:,k), v(:,k), ts, L);
    
    %% DISTANCE TO OBJECTS
    %ob = [object1(:,k+1),object2(:,k+1),object3(:,k+1)];
    d(:,k+1) = distance_objects(ob, h(:,k+1));
    %% NEW VALUES OPTIMAL CONTROL
    vc = [control(2:end,:);control(end,:)];
    H0 = [H0(2:end,:);H0(end,:)];
    
    %% SAMPLE TIME
    t_sample(k) = toc;
    toc;
end

%% AVERAGE TIME
trms = sum(t_sample)/(length(t_sample))
xnorm = norm(he(1,:),2)
ynorm = norm(he(2,:),2)
znorm = norm(he(3,:),2)
psinorm = norm(he(4,:),2)
%%
close all; paso=1; 
%a) Parámetros del cuadro de animación
figure
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [2 2]);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperPosition', [0 0 10 4]);
myVideo = VideoWriter('myVideoFile'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)
luz = light;
luz.Color=[0.65,0.65,0.65];
luz.Style = 'infinite';
%b) Dimenciones del Robot
   Drone_Parameters(0.02);
%c) Dibujo del Robot    
    G2=Drone_Plot_3D(h(1,1),h(2,1),h(3,1),0,0,h(4,1));hold on

    plot3(h(1,1),h(2,1),h(3,11),'--','Color',[56,171,217]/255,'linewidth',1.3);hold on,grid on   
    plot3(hxd(1),hyd(1),hzd(1),'Color',[32,185,29]/255,'linewidth',1.3);
    G_ob1 = plot3(object1(1,1), object1(2,1), object1(3,1),'*','Color',[15,12,15]/255,'linewidth',1.3);
    G_ob2 = plot3(object2(1,1), object2(2,1), object2(3,1),'*','Color',[15,12,15]/255,'linewidth',1.3);
    G_ob3 = plot3(object3(1,1), object3(2,1), object3(3,1),'*','Color',[15,12,15]/255,'linewidth',1.3);

    
view(20,15);
for k = 1:10:length(t)-N
    drawnow
    delete(G2);
%     delete(G_ob1);
%     delete(G_ob2);
%     delete(G_ob3);
   
    G2=Drone_Plot_3D(h(1,k),h(2,k),h(3,k),0,0,h(4,k));hold on
%     G_ob1 = plot3(object1(1,k), object1(2,k), object1(3,k),'*','Color',[15,12,15]/255,'linewidth',1.3);
%     G_ob2 = plot3(object2(1,k), object2(2,k), object2(3,k),'*','Color',[15,12,15]/255,'linewidth',1.3);
%     G_ob3 = plot3(object3(1,k), object3(2,k), object3(3,k),'*','Color',[15,12,15]/255,'linewidth',1.3);

    grid('minor')
    grid on;
    plot3(hxd(1:k),hyd(1:k),hzd(1:k),'Color',[32,185,29]/255,'linewidth',1.3);
    plot3(h(1,1:k),h(2,1:k),h(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.3);
    
    legend({'$\eta$','$\eta_{ref}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
    legend('boxoff')
    %title('$\textrm{Movement Executed by the Aerial Robot}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo)
print -dpng SIMULATION_1
print -depsc SIMULATION_1

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(he)),he(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(he)),he(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(he)),he(3,:),'Color',[26,115,160]/255,'linewidth',1);hold on;
plot(t(1:length(he)),he(4,:),'Color',[83,57,217]/255,'linewidth',1);hold on;
grid('minor')
grid on;
legend({'$\tilde{\eta_{x}}$','$\tilde{\eta_{y}}$','$\tilde{\eta_{z}}$','$\tilde{\eta_{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
%title('$\textrm{Evolution of Control Errors}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(ul)),ul,'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul)),um,'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(ul)),un,'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(ul)),w,'Color',[83,57,217]/255,'linewidth',1); hold on

% plot(t(1:length(v)),v(1,:),'--','Color',[226,76,44]/255,'linewidth',1); hold on
% plot(t(1:length(v)),v(2,:),'--','Color',[46,188,89]/255,'linewidth',1); hold on
% plot(t(1:length(v)),v(3,:),'--','Color',[26,115,160]/255,'linewidth',1); hold on
% plot(t(1:length(v)),v(4,:),'--','Color',[83,57,217]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu_{lref}$','$\mu_{mref}$','$\mu_{nref}$','$\omega_{ref}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
% legend({'$\mu_{lc}$','$\mu_{mc}$','$\mu_{nc}$','$\omega_{c}$','$\mu_{l}$','$\mu_{m}$','$\mu_{n}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
%title('$\textrm{Control Values}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(t_sample)),t_sample,'Color',[108,105,105]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$t_{s}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
%title('$\textrm{Control Values}$','Interpreter','latex','FontSize',9);
ylabel('$[s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);  
print -dpng SAMPLE_TIME
print -depsc SAMPLE_TIME

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(d)),d(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(d)),d(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(d)),d(3,:),'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(d)),r(1,1:length(d)),'--','Color',[15,12,15]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$l_{obs_1}$','$l_{obs_2}$','$l_{obs_3}$','$r_{obs}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff');
ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
