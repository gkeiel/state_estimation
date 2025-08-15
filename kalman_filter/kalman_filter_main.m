clc; clear; close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SYSTEM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% import system
% x'(t) = Ax(t) +Bu(t) +Ew(t)
%  y(t) = Cx(t) +v(t)
[ sys ] = sys_RC;
%[ sys ] = sys_RLC;

% discrete-time model equivalent
% x(k+1) = Fx(k) +Gu(k) +Jw(k)
%   z(k) = Hx(k) +v(k)
T        = 1/1000;
sysd     = c2d( sys,T,'zoh' );

% define input and disturbance signals
Tf    = 0.10;                               % simulation time                
[u,t] = gensig('square',Tf/2,Tf,T);         % input and time
q     = 0.1;                                % variance of process
r     = 0.4;                                % variance of measurement
w     = q*randn(length(t),1);               % gaussian noise with covariance Q
v     = r*randn(length(t),1);               % gaussian noise with variance R

% simulate system WITH process and measurement NOISE
[z,t,x_] = lsim( sysd, [u w], t );
z        = z +v;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_0 = 0.1*ones( length(sys.A),1 );          % initial condition
P_0 = 10*diag( ones( length(sys.A),1 ) );   % initial covariance
[ x, y, K_m, P_m ] = kalman_filter( sysd, u, w', z, t, x_0, P_0, r, q );


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RESULTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[y_o,t_o,x_o] = lsim( sys, [u 0*w], t);     % IDEAL behavior for comparison

figure(1)
for k = 1:length(sys.A)
    subplot( round((length(sys.A)+1)/2 ),2,k)
    plot(t,x_o(:,k),'k','linewidth',2); hold on;
    plot(t,x_(:,k),'bo:','linewidth',2); 
    stairs(t,x(:,k),'r','linewidth',1.5);
    xlabel('Time (s)'); ylabel('Amplitude'); title(['State x_' num2str(k) '(t)']); grid on;
    legend('ideal','real','estimated'); hold off;
end

subplot( round((length(sys.A)+1)/2 ),2,k+1)
plot(t,y_o,'k','linewidth',2); hold on;
plot(t,z,'bo:','linewidth',2);
stairs(t,y,'r','linewidth',1.5);
xlabel('Time (s)'); ylabel('Amplitude'); title('Output y(t)'); grid on;
legend('ideal','measured','estimated');