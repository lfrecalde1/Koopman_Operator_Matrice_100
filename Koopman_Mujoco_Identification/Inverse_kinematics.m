function [u] = Inverse_kinematics(h,hd, k1, k2)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
hnew = [h(1:3);h(10)];
he = hd - hnew;
psi = hnew(4);
K2 = k2*eye(4);
K1 = k1*eye(4);

J = [cos(psi), -sin(psi), 0, 0;...
     sin(psi), cos(psi), 0 ,0;...
     0, 0, 1, 0;...
     0, 0, 0, 1];
 
u = inv(J)*(K2*tanh(inv(K2)*K1*he)); 

end

