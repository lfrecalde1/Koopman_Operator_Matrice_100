 function [f,solver,args] = mpc_drone(bounded, N, A, B, G, ts)

addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');
import casadi.*;

%% Definicion de las restricciones en las acciones de control
uz_max = bounded(1); 
uz_min = bounded(2);

phi_max = bounded(3);
phi_min = bounded(4);

theta_max = bounded(5);
theta_min = bounded(6);

w_max = bounded(7); 
w_min = bounded(8);

number_states = size(A, 1);
number_inputs = size(B, 2);

%% Generacion de las variables simbolicas de los estados del sistema
x = SX.sym('x', number_states, 1); 
u = SX.sym('u', 4, 1);
%% Definicion de cuantos estados en el sistema
n_states = length(x);

u_aux = [0; 0; u];
%% Defincion de cuantas acciones del control tiene el sistema
n_control = length(u);

R_t = [Rot_zyx(x(10:12)), zeros(3, 3);...
           zeros(3,3), eye(3, 3)];

rhs=(A*x + B*R_t*u_aux + G);

%% Definicion de kas funciones del sistema
f = Function('f',{x,u},{rhs}); 
U = SX.sym('U',n_control,N);
P = SX.sym('P',n_states + N*(n_states));
%% vector que incluye el vector de estados y la referencia
X = SX.sym('X',n_states,(N+1));

%% Vector que representa el problema de optimizacion
g = [];  % restricciones de estados del problema  de optimizacion

%%EMPY VECTOR ERRORS
he = [];
he_angular = [];
%% EMPY VECTOR CONTROL VALUES
u = [];

%% INITIAL CONDITION ERROR
st  = X(:,1); % initial state
g = [g;X(:,1)-P(1:27)]; % initial condition constraints

%% Definicon del bucle para optimizacion a los largo del tiempo
for k = 1:N
    st = X(:,k);  con = U(:,k);

    %% Funcion costo a minimizar 
    he = [he;X(1:3,k)-P(27*k+1:27*k+3)];
    he_angular = [he_angular;X(15,k)-P(27*k+15)];
    u = [u;con];
    
    %% Actualizacion del sistema usando Euler runge kutta
    st_next = X(:,k+1);
    st_next_RK4= f(st, con); % new 
    
    %% Restricciones del sistema se =basan en el modelo del sistema
    g = [g;st_next-st_next_RK4]; 
end


%% Cost final 
Q = 1*eye(size(he,1));
Q_1 = 0.1*eye(size(he_angular,1));
R = 0.01*eye(size(u,1));
%% FINAL COST
obj = he'*Q*he+u'*R*u + he_angular'*Q_1*he_angular;

% for k =1:N-1
%     g = [g; U(1,k)-U(1,k+1) - 0.1];
%     g = [g; U(1,k+1)-U(1,k) - 0.01];
%     
%     g = [g; U(2,k)-U(2,k+1) - 0.01];
%     g = [g; U(2,k+1)-U(2,k) - 0.01];
%     
%     g = [g; U(3,k)-U(3,k+1) - 0.01];
%     g = [g; U(3,k+1)-U(3,k) - 0.01];
%     
%     g = [g; U(4,k)-U(4,k+1) - 0.1];
%     g = [g; U(4,k+1)-U(4,k) - 0.1];
%     
% end
% se crea el vector de desiscion solo de una columna
OPT_variables = [reshape(X,27*(N+1),1);reshape(U,4*N,1)];

nlprob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlprob,opts);

args = struct;

args.lbg(1:27*(N+1)) = 0;  %-1e-20  %Equality constraints
args.ubg(1:27*(N+1)) = 0;  %1e-20   %Equality constraints

% args.lbg(27*(N+1)+1 : 27*(N+1) +1 + 27*(N-1)) = -inf; % inequality constraints
% args.ubg(27*(N+1)+1 : 27*(N+1) +1 + 27*(N-1)) = 0; % inequality constraints


args.lbx(1:27:27*(N+1),1) = -inf; %state x lower bound
args.ubx(1:27:27*(N+1),1) = inf;  %state x upper bound

args.lbx(2:27:27*(N+1),1) = -inf; %state y lower bound
args.ubx(2:27:27*(N+1),1) = inf;  %state y upper bound

args.lbx(3:27:27*(N+1),1) = -inf; %state z lower bound
args.ubx(3:27:27*(N+1),1) = inf;  %state z upper bound

args.lbx(4:27:27*(N+1),1) = -inf; %state theta lower bound
args.ubx(4:27:27*(N+1),1) = inf;  %state theta upper bound

args.lbx(5:27:27*(N+1),1) = -inf; %state x lower bound
args.ubx(5:27:27*(N+1),1) = inf;  %state x upper bound

args.lbx(6:27:27*(N+1),1) = -inf; %state y lower bound
args.ubx(6:27:27*(N+1),1) = inf;  %state y upper bound

args.lbx(7:27:27*(N+1),1) = -inf; %state z lower bound
args.ubx(7:27:27*(N+1),1) = inf;  %state z upper bound

args.lbx(8:27:27*(N+1),1) = -inf; %state theta lower bound
args.ubx(8:27:27*(N+1),1) = inf;  %state theta upper bound

args.lbx(9:27:27*(N+1),1) = -inf; %state theta lower bound
args.ubx(9:27:27*(N+1),1) = inf;  %state theta upper bound

args.lbx(10:27:27*(N+1),1) = -inf; %state x lower bound
args.ubx(10:27:27*(N+1),1) = inf;  %state x upper bound

args.lbx(11:27:27*(N+1),1) = -inf; %state y lower bound
args.ubx(11:27:27*(N+1),1) = inf;  %state y upper bound

args.lbx(12:27:27*(N+1),1) = -inf; %state z lower bound
args.ubx(12:27:27*(N+1),1) = inf;  %state z upper bound

args.lbx(13:27:27*(N+1),1) = -inf; %state theta lower bound
args.ubx(13:27:27*(N+1),1) = inf;  %state theta upper bound

args.lbx(14:27:27*(N+1),1) = -inf; %state x lower bound
args.ubx(14:27:27*(N+1),1) = inf;  %state x upper bound

args.lbx(15:27:27*(N+1),1) = -inf; %state y lower bound
args.ubx(15:27:27*(N+1),1) = inf;  %state y upper bound

args.lbx(16:27:27*(N+1),1) = -inf; %state z lower bound
args.ubx(16:27:27*(N+1),1) = inf;  %state z upper bound

args.lbx(17:27:27*(N+1),1) = -inf; %state theta lower bound
args.ubx(17:27:27*(N+1),1) = inf;  %state theta upper bound

args.lbx(18:27:27*(N+1),1) = -inf; %state z lower bound
args.ubx(18:27:27*(N+1),1) = inf;  %state z upper bound

args.lbx(19:27:27*(N+1),1) = -inf; %state theta lower bound
args.ubx(19:27:27*(N+1),1) = inf;  %state theta upper bound

args.lbx(20:27:27*(N+1),1) = -inf; %state x lower bound
args.ubx(20:27:27*(N+1),1) = inf;  %state x upper bound

args.lbx(21:27:27*(N+1),1) = -inf; %state y lower bound
args.ubx(21:27:27*(N+1),1) = inf;  %state y upper bound

args.lbx(22:27:27*(N+1),1) = -inf; %state z lower bound
args.ubx(22:27:27*(N+1),1) = inf;  %state z upper bound

args.lbx(23:27:27*(N+1),1) = -inf; %state theta lower bound
args.ubx(23:27:27*(N+1),1) = inf;  %state theta upper bound

args.lbx(24:27:27*(N+1),1) = -inf; %state x lower bound
args.ubx(24:27:27*(N+1),1) = inf;  %state x upper bound

args.lbx(25:27:27*(N+1),1) = -inf; %state y lower bound
args.ubx(25:27:27*(N+1),1) = inf;  %state y upper bound

args.lbx(26:27:27*(N+1),1) = -inf; %state z lower bound
args.ubx(26:27:27*(N+1),1) = inf;  %state z upper bound

args.lbx(27:27:27*(N+1),1) = -inf; %state theta lower bound
args.ubx(27:27:27*(N+1),1) = inf;  %state theta upper bound




%% Definicion de las restricciones de las acciones de control del sistema
args.lbx(27*(N+1)+1:4:27*(N+1)+4*N,1) = uz_min;  %
args.ubx(27*(N+1)+1:4:27*(N+1)+4*N,1) = uz_max;  %

args.lbx(27*(N+1)+2:4:27*(N+1)+4*N,1) = theta_min;  %
args.ubx(27*(N+1)+2:4:27*(N+1)+4*N,1) = theta_max;  % 

args.lbx(27*(N+1)+3:4:27*(N+1)+4*N,1) = phi_min;  %
args.ubx(27*(N+1)+3:4:27*(N+1)+4*N,1) = phi_max;  %

args.lbx(27*(N+1)+4:4:27*(N+1)+4*N,1) = w_min;  %
args.ubx(27*(N+1)+4:4:27*(N+1)+4*N,1) = w_max;  %

end