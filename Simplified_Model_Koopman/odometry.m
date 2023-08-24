function [h,hp,u] = odometry(odom)
%UNTITLED Odometry of the Aerial Vehicle
% Read odometry values from ros
odomdata = receive(odom,3);

pose = odomdata.Pose.Pose;
vel = odomdata.Twist.Twist;

quat = odomdata.Pose.Pose.Orientation;

orientacion_aux = (quat2eul([quat.W quat.X quat.Y quat.Z],'ZYX'))';


% Get values of position an orientation
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;


% Get values of linear an angular velocities
vx = vel.Linear.X;
vy = vel.Linear.Y;
vz = vel.Linear.Z;


wx = vel.Angular.X;
wy = vel.Angular.Y;
wz = vel.Angular.Z;

% create vector of position and angular states
h = [x;...
     y;...
     z;...
     quat.W;...
     quat.X;...
     quat.Y;...
     quat.Z;...
     orientacion_aux(3);...
     orientacion_aux(2);...
     orientacion_aux(1)];
R = Rot_zyx([orientacion_aux(3), orientacion_aux(2), orientacion_aux(1)]);

V_w = R*[vx;vy;vz];
% Create vector of linear an angular velocities
hp = [V_w(1);...
      V_w(2);...
      V_w(3);...
      wx;...
      wy;...
      wz];
u = [vx;vy;vz];
end

