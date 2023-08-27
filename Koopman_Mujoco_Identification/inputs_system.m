function [F,T] = inputs_system(inputs)
%UNTITLED Odometry of the Aerial Vehicle
% Read odometry values from ros
inputsdata = receive(inputs,3);


Force_lineal = inputsdata.Linear;
Torques_angular = inputsdata.Angular;

%angles = quat2eul([quat.W quat.X quat.Y quat.Z]);

% Get lineal forces
fx = Force_lineal.X;
fy = Force_lineal.Y;
fz = Force_lineal.Z;


% Get angulat torques
tx = Torques_angular.X;
ty = Torques_angular.Y;
tz = Torques_angular.Z;


% create vector of Forces
F = [fx;...
     fy;...
     fz];

% Create vector of Torques
T = [tx;...
     ty;...
     tz];


end