clc; clear; close all;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SYSTEM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% import discrete-time model
% x(k+1) = f(x(k),u(k)) +w(k)
%   y(k) = h(x(t)) +v(k)
[ sysd ] = nl_sys_2nd_order;

% define input and disturbance signals
T     = sysd.T;
Tf    = 0.5;                                % simulation time                
[u,t] = gensig('square',Tf/2,Tf,T);         % input signal and time
u     = 5*u;                                % input amplitude
Q     = 0.5;                                % variance of process noise
R     = 0.5;                                % variance of measurement noise
w     = sqrt(Q)*randn(length(t),1);         % gaussian noise with covariance Q
v     = sqrt(R)*randn(length(t),1);         % gaussian noise with variance R

% simulate time response of TRUE system with process NOISE
n     = sysd.n;
x_    = ones( n,1 );
f     = sysd.f;
h     = sysd.h;
for k = 1:length(t)-1
    x_(:,k+1) = f(x_(:,k), u(k)) +w(k);
    y(k+1)    = h(x_(:,k+1));
end
x_ = x_.';
z  = y' +v;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_0 = 10*ones( n,1 );                       % initial condition
P_0 = 10*diag( ones(n,1) );                 % initial covariance
% obtain estimated states and output
[ x, y_hat, e, K_m, P_m ] = extended_kalman_filter( sysd, u, w', z, t, x_0, P_0, R, Q );


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RESULTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
for k = 1:n
    subplot( round((n+1)/2), 2, k )
    plot(t,x_(:,k),'k','linewidth',2);
    hold on;
    stairs(t,x(:,k),'r','linewidth',1.5);
    xlabel('Time (s)'); ylabel('Amplitude'); title(['State x_' num2str(k) '(t)']); grid on;
    legend('real','estimated'); hold off;
end

subplot( round((n+1)/2), 2, k+1 )
plot(t,y,'k','linewidth',2);
hold on;
plot(t,z,'bo:','linewidth',2);
stairs(t,y_hat,'r','linewidth',1.5);
xlabel('Time (s)'); ylabel('Amplitude'); title('Output y(t)'); grid on;
legend('real','measured','estimated');