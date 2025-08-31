function [ sysd ] = nl_sys_1st_order

% system order
n = 2;

% nonlinear dynamics
T = 0.01;
f = @(x,u) [ x(1)+T*(-x(1)^3 +u) ];  
h = @(x)   [ x(1)^2 ];                      

% jacobians
f_jac = @(x,u) [ 1+T*(-3*x(1)^2) ];     
h_jac = @(x)   [ 2*x(1) ];

sysd = struct('f',f,'h',h,'f_jac',f_jac,'h_jac',h_jac,'T',T,'n',n);