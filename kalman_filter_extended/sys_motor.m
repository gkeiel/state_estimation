function [ sys ] = sys_motor

% DC motor model
J   = 3.22e-6;   %kg*m^2
b   = 3.5e-6;    %N*m*s
k_b = 0.0274;    %V/rad/s
k_t = 0.0274;    %N*m/A
R_a = 4;         %Ohm
L_a = 2.75e-6;   %H

A = [0      1          0;
     0     -b/J        k_b/J;
     0     -k_t/L_a   -R_a/L_a];
B = [0;     0;         1/L_a];
C = [1      0          0];
D = 0;
E = [0      0          0;
     0      0          0;
     0      0          1/L_a];
sys = ss(A,[B E],C,[D zeros(1,length(B))]);