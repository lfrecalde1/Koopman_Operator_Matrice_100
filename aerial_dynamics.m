function [aceleration] = aerial_dynamics(v, h, u, x)
                                             
%% Parameters of the system 
m = x(1);
g = 9.81;
aux = [0; 0; 1];
euler = h(1:3);

Friction_matrix= [x(2), 0, 0;...
                  0, x(6), 0;...
                  0, 0, x(10)];
              
%% Input of the system
T = [0; 0; u];
velocity = v;
R = Rot_zyx(euler);

%% Aproximation
factor = R*(T)/m;
% factor(3) = factor(3)*0.97;
% factor(2) = factor(2)*0.8;
% factor(1) = factor(1)*0.9;
aceleration =  factor - g*aux;

end