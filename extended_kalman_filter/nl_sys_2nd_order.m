function [ sysd ] = nl_sys_2nd_order

% system order
n = 2;

% nonlinear dynamics
T = 0.01;
f = @(x,u) [ x(1)+T*x(2);
             x(2)+T*(-sin(x(1))+u) ];  
h = @(x)   [ x(1)^2 ];                      

% jacobians
f_jac = @(x,u) [ 1,           T;
                 T*cos(x(1)), 1 ];     
h_jac = @(x)   [ 2*x(1), 0 ];

sysd = struct('f',f,'h',h,'f_jac',f_jac,'h_jac',h_jac,'T',T, 'n',n);