function [ sys ] = sys_RLC

% LC filter model
r_1 = 0.015;
L_1 = 0.001;
C_1 = 0.0003;
Y   = 0.152;

A = [-r_1/L_1  -1/L_1;
      1/C_1    -Y/C_1];
B = [1/L_1;    0];
C = [0         1];
D = 0;
E = B;
sys = ss(A,[B E],C,[D D]);