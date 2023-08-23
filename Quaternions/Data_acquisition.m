%% Syten Data Adquisition Mujoco

%% Clear Variables
clc, clear all, close all;

%% Definition Sample Time
ts = 0.03;
tfinal = 20;
t = (0:ts:tfinal);

%% Load Values of the desired Signals
n = 12;
Signals = Velocities_data(12,t,1);

%% Reference Signals
ul_ref = Signals(1,:);
um_ref = Signals(2,:);
un_ref = Signals(3,:);
w_ref = Signals(4,:);

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

%% Init System
for i = 1:200
    tic;
    send_velocities(robot_references, velmsg, [0,0, 0, 0, 0 , 0]);
    [aux_h, aux_hp, aux_u] = odometry(odom);
    while(toc<ts)
    end
end

for k=1:1:length(t)
    tic; 
    %% Send control values to the robot
    send_velocities(robot_references, velmsg, [ul_ref(k), um_ref(k), un_ref(k), 0, 0 , w_ref(k)]);
    
    
    while(toc<ts)
        [F(:, k), T(:, k)] =  inputs_system(inputs);
        [omega_ref(:, k)] = inputs_system_rate(inputs);
    end
    %% Getas data from the drone
    [h(:, k+1), hp(:,k+1), u(:, k+1)] = odometry(odom);
    t_real(k) = toc;
end
send_velocities(robot_references, velmsg, [0, 0, 0, 0, 0 , 0])
rosshutdown;

T_ref = [F(3, :);...
         omega_ref(:, :);...
         w_ref(:,:)];

vz_d = un_ref(:, :);
%% Save Data System
%save("Data_mujoco.mat", "ts", "t", "ul_ref", "um_ref", "un_ref", "w_ref", "h", "hp", "T", "F", "omega_ref")
save("Data_mujoco_2.mat", "ts", "t", "T_ref", "h", "hp", "vz_d", "u", "ul_ref", "um_ref", "un_ref", "w_ref")