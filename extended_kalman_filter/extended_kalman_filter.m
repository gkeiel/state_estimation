function [ x, y, e, K_m, P_m ] = extended_kalman_filter( sysd, u, w, z, t, x_0, P_0, R, Q )

% inputs:
% sysd   : discrete-time nonlinear model 
% u      : input signal
% z      : measurements
% t      : time vector
% x_0    : initial state estimate
% P_0    : initial covariance
% R      : measurement noise covariance
% Q      : process noise covariance

% discrete-time nonlinear model
f     = sysd.f;     % state transition function handle f(x,u)
h     = sysd.h;     % measurement function handle h(x)
f_jac = sysd.f_jac; % jacobian of f function handle
h_jac = sysd.h_jac; % jacobian of h function handle

% initialization
N = length(t);
k = 1;
x = x_0;
P = P_0;
I = eye(length(x_0));
K_m = [];
P_m = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%% kalman filter %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n = 1:N-1

    % 1. prediction
    x(:,k+1) = f(x(:,k), u(k)) +w(k);      % x[k+1|k] 'a priori' state prediction
    F        = f_jac(x(:,k), u(k));        % jacobian at current state
    P        = F*P*F' +Q;                  % P[k+1|k] 'a priori' covariance prediction

    % 2. correction
    H        = h_jac(x(:,k+1));            % jacobian of measurement
    K        = P*H'/( H*P*H' +R );         % computes optimal gain
    e(k+1)   = z(k+1) -h(x(:,k+1));        % measurement residual (innovation)
    x(:,k+1) = x(:,k+1) +K*e(k+1);         % 'a posteriori' state estimative 
    P        = (I -K*H)*P;                 % 'a posteriori' covariance
    y(k+1)   = h(x(:,k+1));                % estimated output
                              
    K_m = [K_m K];                         % stores current K
    P_m = [P_m P];                         % stores current P
    k   = k +1;
end

x = x.';
end