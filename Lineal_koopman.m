%% Data Figure
%% Clear variables
clc, clear all, close all;

%% Load information
load("h_2.mat");
load("hp_2.mat");
load("T_ref_2.mat");
load("t_2.mat");
des = 1;

%% Get the data of the system
[Data_2_X_k, Data_2_X_1, Data_2_U_1, euler] = get_data_simple_velocities(h, hp, t, T_ref);


%% State K
X1 = [Data_2_X_1];

%% State K+1
X2 = [Data_2_X_k];
 
%% Input K
Gamma = [Data_2_U_1/100];

%% Lifted Matrices
n = 3; 

%% Matrix values for RBF
% RBF 1 Centers
Nrbf = 2;            

cent_l = rand(n, 5)*2 - 1;   
cent_lz = rand(n, 2)*2 - 1; 
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
 
% Parameter matrices
alpha = 0.2;
beta = 0.5;
% Parametros del optimizador
options = optimset('Display','iter',...
    'TolFun', 1e-8,...
    'MaxIter', 60000,...
    'Algorithm', 'active-set',...
    'FinDiffType', 'forward',...
    'RelLineSrchBnd', [],...
    'RelLineSrchBndDuration', 1,...
    'TolConSQP', 2e-8);

%% Initial Condition Optimization problem
x0=ones(1,285).*rand(1,285);
f_obj1 = @(x)  funcion_costo_DMD(x, length(t), X1, X2, Gamma, alpha, beta, euler);
tic

%% Optimization Problem
x = fmincon(f_obj1,x0,[],[],[],[],[],[],[],options);
chi = x;
toc

%
 
%% Model Of the system
A_l = [ x(1), x(2),  x(3),  x(4),  x(5),  x(6), x(7),  x(8),  x(9), x(10), x(11), x(12), x(13), x(14), x(15);...
       x(16), x(17), x(18), x(19), x(20), x(21), x(22), x(23), x(24), x(25), x(26), x(27), x(28), x(29), x(30);...
       x(31), x(32), x(33), x(34), x(35), x(36), x(37), x(38), x(39), x(40), x(41), x(42), x(43), x(44), x(45);...
       x(46), x(47), x(48), x(49), x(50), x(51), x(52), x(53), x(54), x(55), x(56), x(57), x(58), x(59), x(60);...
       x(61), x(62), x(63), x(64), x(65), x(66), x(67), x(68), x(69), x(70), x(71), x(72), x(73), x(74), x(75);...
       x(76), x(77), x(78), x(79), x(80), x(81), x(82), x(83), x(84), x(85), x(86), x(87), x(88), x(89), x(90);...
       x(91), x(92), x(93), x(94), x(95), x(96), x(97), x(98), x(99), x(100), x(101), x(102), x(103), x(104), x(105);
       x(106), x(107), x(108), x(109), x(110), x(111), x(112), x(113), x(114), x(115), x(116), x(117), x(118), x(119), x(120);...
       x(121), x(122), x(123), x(124), x(125), x(126), x(127), x(128), x(129), x(130), x(131), x(132), x(133), x(134), x(135);...
       x(136), x(137), x(138), x(139), x(140), x(141), x(142), x(143), x(144), x(145), x(146), x(147), x(148), x(149), x(150);...
       x(151), x(152), x(153), x(154), x(155), x(156), x(157), x(158), x(159), x(160), x(161), x(162), x(163), x(164), x(165);...
       x(166), x(167), x(168), x(169), x(170), x(171), x(172), x(173), x(174), x(175), x(176), x(177), x(178), x(179), x(180);...
       x(181), x(182), x(183), x(184), x(185), x(186), x(187), x(188), x(189), x(190), x(191), x(192), x(193), x(194), x(195);...
       x(196), x(197), x(198), x(199), x(200), x(201), x(202), x(203), x(204), x(205), x(206), x(207), x(208), x(209), x(210);...
       x(211), x(212), x(213), x(214), x(215), x(216), x(217), x(218), x(219), x(220), x(221), x(222), x(223), x(224), x(225)];
 
 
   
B_l = [x(226), x(227), x(228);...
       x(229), x(230), x(231);...
       x(232), x(233), x(234);...
       x(235), x(236), x(237);...
       x(238), x(239), x(240);...
       x(241), x(242), x(243);...
       x(244), x(245), x(246);...
       x(247), x(248), x(249);...
       x(250), x(251), x(252);...
       x(253), x(254), x(255);...
       x(256), x(257), x(258);...
       x(259), x(260), x(261);...
       x(262), x(263), x(264);...
       x(265), x(266), x(267);...
       x(268), x(269), x(270)];
   
G_l = [x(271);...
       x(272);...
       x(273);...
       x(274);...
       x(275);...
       x(276);...
       x(277);...
       x(278);...
       x(279);...
       x(280);...
       x(281);...
       x(282);...
       x(283);...
       x(284);...
       x(285)];
 
C_l = eye(3,15);

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
    
    R = Rot_zyx(euler(:, k));
   
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
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(salida_real(2,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(salida_es(2,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${v_y}$','$\hat{v_y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(salida_real(3,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(salida_es(3,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${v_z}$','$\hat{v_z}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

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

figure
imagesc(A_l);

figure
imagesc(B_l);

figure
imagesc(G_l);

save("matrices_lineal.mat", "A_l", "B_l", "G_l", "C_l", "cent_l", "cent_lz")