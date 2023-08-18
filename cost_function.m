function [cost] = cost_function(x, N, u_ref, u, u_p, euler)
                                             
he = [];
m = x(1);
g = 9.81;
aux = [0; 0; 1];

Friction_matrix= [x(2), 0, 0;...
                  0, x(6), 0;...
                  0, 0, x(10)];
for k=1:N
    %% Get Values for the PID
    %% Input of the system
    trust = u_ref(1,k);
    T = [0; 0; trust];
    
    acceleration = u_p(:,k);
    velocity = u(:, k);
    R = Rot_zyx(euler);
    
    %% Aproximation
    
    aprox =  R*T - m*g*aux;  

    %% Error Vector
    he = [he; acceleration*m - aprox];
end
cost = norm(he,2);
end