function [ sys ] = sys_cruise

% cruise model
m = 1000; % kg
b = 50;   % Ns/m

A = -b/m;
B = 1/m;
C = 1;
D = 0;
E = B;
n = size(E);
sys = ss(A,[B E],C,[D zeros(1,n(2))]);