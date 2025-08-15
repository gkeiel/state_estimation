function [ sys ] = sys_pendulum

% pendulum model
M = 0.5;   %kg
m = 0.2;   %kg
b = 0.1;   %N/m/s
l = 0.4;   %m
g = 9.8;   %m/s
I = 0.006; %kg*m^2

d = I*(M +m) +M*m*l^2;
A = [0      1               0               0;
     0     -(I+m*l^2)*b/d   m^2*g*l^2/d     0;
     0      0               0               1;
     0     -m*l*b/d         m*g*l*(M+m)/d   0];
B = [0;    (I+m*l^2)/d;     0;              m*l/d];
C = [0      0               0               1];
D = 0;
E = [0     0              0              0;
     0     (I+m*l^2)/d    0              0;
     0     0              0              0;
     0     0              0              m*l/d];
sys = ss(A,[B E],C,[D zeros(1,length(B))]);