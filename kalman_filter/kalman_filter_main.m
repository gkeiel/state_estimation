clc; clear; close all;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SYSTEM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% import system
% dx/dt = Ax(t) +Bu(t) +Ew(t)
%  y(t) = Cx(t)
[ sys ] = sys_RC;

% discrete-time model equivalent (Anderson and Moore, 1979)
% x(k+1) = Fx(k) +Gu(k) +Jw(k)
%   z(k) = Hx(k) +v(k)
T        = 1/1000;
sysd     = c2d( sys,T,'zoh' );

% define input and disturbance signals
Tf    = 0.10;                               % simulation time                
[u,t] = gensig('square',Tf/3,Tf,T);         % input signal and time
u     = 5*u;                                % input amplitude
Q     = 0.5;                                % variance of process noise
R     = 0.5;                                % variance of measurement noise
w     = sqrt(Q)*randn(length(t),1);         % gaussian noise with covariance Q
v     = sqrt(R)*randn(length(t),1);         % gaussian noise with variance R

% simulate time response of TRUE system with process NOISE
[y,t,x_] = lsim( sys, [u w], t);
z        = y +v;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n   = length(sys.A);
x_0 = 1*ones( n,1 );                        % initial condition
P_0 = 10*diag( ones(n,1) );                 % initial covariance
% obtain estimated states and output
[ x, y_hat, e, K_m, P_m ] = kalman_filter( sysd, u, w', z, t, x_0, P_0, R, Q );


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RESULTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
for k = 1:n
    subplot( round((n+1)/2), 2, k )
    plot(t,x_(:,k),'k','linewidth',2);
    hold on;
    stairs(t,x(:,k),'r','linewidth',1.5);
    xlabel('Time (s)'); ylabel('Amplitude'); title(['State x_' num2str(k) '(t)']); grid on;
    legend('real','estimated');
end

subplot( round((n+1)/2), 2, k+1 )
plot(t,y,'k','linewidth',2);
hold on;
plot(t,z,'bo:','linewidth',2);
stairs(t,y_hat,'r','linewidth',1.5);
xlabel('Time (s)'); ylabel('Amplitude'); title('Output y(t)'); grid on;
legend('real','measured','estimated');

% B. D. Anderson, and J. B. Moore, “Optimal filtering,” Prentice–Hall, New Jersey, 1979.