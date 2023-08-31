function [H0, control] = NMPC(h, hd, k, H0, vc, args, solver ,N)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
H = [h];
args.p(1:27) = H;

for i = 1:N
    args.p(27*i+1:27*i+27)=[hd(:,k+i)];
end


args.x0 = [reshape(H0',27*(N+1),1);reshape(vc',size(vc,2)*N,1)]; % initial value of the optimization variables

sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

control = reshape(full(sol.x(27*(N+1)+1:end))',4,N)';
H0 = reshape(full(sol.x(1:27*(N+1)))',27,N+1)';
end

