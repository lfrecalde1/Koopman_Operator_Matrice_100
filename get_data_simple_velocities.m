function [X2, X1, Gamma, euler] = get_data_simple_velocities(h, hp, t, u_ref)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
%% Load Data experiment 1
des =1;

%% Load Data System Pose
h = h(:, des:end-1);

%% Load Data Velocities
hp = hp(:, des:end-1);
p = hp(4, :);
q = hp(5, :);
r = hp(6, :);

%% Load Time
t = t(:,des:end);

%% Length Simulation
N = length(t);

%% Reference Angles
phi_ref = u_ref(2, :);
theta_ref = u_ref(3, :);
euler_ref = [phi_ref;...
             theta_ref];
w_ref = u_ref(4, :);
%% Real Angles System
phi = h(8, :);
theta = h(9,:);
psi = h(10, :);

euler = [phi;...
         theta;...
          psi];

%% Angles velocities
for k =1:length(hp)
[euler_p(:, k)] = Euler_p(hp(4:6, k),h(8:10, k));
end
%% Angular Velocities Body euler
phi_p = euler_p(1, :);
theta_p = euler_p(2, :);
psi_p = euler_p(3, :);

%% generalized Data system
X = [hp(1:3,:)];

%% Control Signal
U_ref = [0*u_ref(1, :);...
         0*u_ref(1, :);...
         u_ref(1, :)];
     
%% Rearrange data in order to develp DMD ext

X1 = [X(:,1:end-1)];
  
X2 = [X(:,2:end)];

euler = euler(:, 1:end-1);

Gamma = U_ref(:,1:end-1);
end
