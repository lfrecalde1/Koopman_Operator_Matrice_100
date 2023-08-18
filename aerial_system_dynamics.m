function [v] = aerial_system_dynamics(v, h, u, ts, x)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
k1 = aerial_dynamics(v, h, u, x);
k2 = aerial_dynamics(v + ts/2*k1, h, u, x);
k3 = aerial_dynamics(v + ts/2*k2, h, u, x);
k4 = aerial_dynamics(v + ts*k3, h, u, x);

v = v +ts/6*(k1 +2*k2 +2*k3 +k4);
end
