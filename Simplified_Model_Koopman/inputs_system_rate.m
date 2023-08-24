function [W] = inputs_system_rate(inputs)
%UNTITLED Odometry of the Aerial Vehicle
% Read odometry values from ros
inputsdata = receive(inputs,3);


angular_rate_d = inputsdata.Linear;


%angles = quat2eul([quat.W quat.X quat.Y quat.Z]);

% Get lineal forces
wx = angular_rate_d.X;
wy = angular_rate_d.Y;


% Create vector of Torques
W = [wx;...
     wy];
end