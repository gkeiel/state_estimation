function [ x, y, e, K_m, P_m ] = kalman_filter( sysd, u, w, z, t, x_0, P_0, R, Q )

% discrete-time model
F = sysd.a;
G = sysd.b(:,1);
H = sysd.c;
J = sysd.b(:,2:end);

% initialization
k = 1;
x = x_0;                 
P = P_0;
I = diag( ones(length(F),1) );
K_m = [];
P_m = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%% kalman filter %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n = 1:length(t)-1

    % 1. prediction
    x(:,k+1) = F*x(:,k) +G*u(k) +J*w(:,k); % x[k+1|k] 'a priori' predicted state
    P        = F*P*F' +J*Q*J';             % P[k+1|k] 'a priori' predicted covariance matrix

    % 2. correction
    K        = P*H'/( H*P*H' +R );         % computes optimal gain   
    e(k+1)   = z(k+1) -H*x(:,k+1);         % measurement residual (innovation)
    x(:,k+1) = x(:,k+1) +K*e(k+1);         % 'a posteriori' state estimative 
    P        = (I -K*H)*P;                 % 'a posteriori' covariance
    y(k+1)   = H*x(:,k+1);                 % estimated output

    k   = k +1;                              
    K_m = [K_m K];                         % stores current K
    P_m = [P_m P];                         % stores current P            
end

x = transpose(x);
end