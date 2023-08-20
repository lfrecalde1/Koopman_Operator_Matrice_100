clc, clear all, close all
n = 12;
m = 3;

A = rand(n, n)
B = rand(n, n*3)


u = [5;3;4]

x = ones(12,1);

for k =1:length(u)

        factor(:, :, k) = B(1:n, k*n -(n-1):k*n)*x*u(k)


end