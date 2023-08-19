function [x_lift] = liftFun(xx, cent_a, rbf_type)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
x_lift = [xx;
          rbf(xx(4,:),cent_a,rbf_type);
          rbf(xx(5,:), cent_a,rbf_type);
          rbf(xx(6,:), cent_a,rbf_type)];
end

