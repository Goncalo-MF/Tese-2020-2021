clear all, close all, clc

m = 0.25;
M = 0.32;
L = 0.12;
g = -9.8;
d = 1; %damping/fricçao
%syms m M L g d

s = 1; % pendulum up (s=1)

% syms teta
% Cy=cos(teta);
% D = m*L*L*(M+m*(1-Cy^2));

%linearized 

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];
%B=[0; m*L*L/D; 0; s*m*L*Cy/D];

C=[1 0 0 0];

D = 0;

sys=ss(A,B,C,D);

% I=m*L*L/3
% D=(I*(M+m)+M*L*L*m)
% 
% A = [0 1 0 0;
%     0 -d*(I-m*L*L)/D -m*m*L*L*g/D 0;
%     0 0 0 1;
%     0 -s*d*m*L/D -s*(m+M)*m*L*g/D 0];
% 
% B = [0; (I+m*L*L)*d/D; 0; s*m*L/D];

Q = [1000 0 0 0; %Q maior aumenta performance mas aumenta custo também
    0 1 0 0;  %Q para cada variavel, x xdot, o odot
    0 0 10 0; %maior penalizaçao nos tetas porque nao podem ter grandes margem de erro
    0 0 0 10];

R = .00001; % basicamente eletricidade de borla

%%
det(ctrb(A,B));
K = lqr(A,B,Q,R);


%%  Augment system with disturbances and noise

Vd = .001*eye(4);  % disturbance covariance / process noise
                 %(if big, trust y more)
Vn = .001;       % noise covariance / white measurement noise
              %(bad sensor noise can't trust y, relies on model)

%%  Build Kalman filter

[Kf,P,E] = lqe(A,Vd,C,Vd,Vn);  % design Kalman filter

sysKf = ss(A-Kf*C,[B Kf],eye(4),0*[B Kf]);  % Kalman filter estimator

%% Visualizaçao

tspan = 0:0.1:30;
% % ode options

% odeset('relTol'   , 1e-7, 'absTol', realmin);

if(s==-1)
    y0 = [0; 0; 0;0];
    [t,y] = ode45(@(t,y)derivatives_v2(t,y,m,M,L,g,d,K,u),tspan,y0);
    
elseif(s==1)
    y0 = [0; 0; pi; 0];
%     [t,y] = ode45(@(t,y)((A-B*K)*(y-[3; 0; pi; 0])),tspan,y0,gh);
    [t,y] = ode45(@(t,y)derivatives_v2(t,y,m,M,L,g,d,K),tspan,y0);
    
else   
end

% for k=1:100:length(t)
%     drawcart(y(k,:),m,M,L);
% end


subplot(2,1,1), plot(y(:,2))
hold on
plot(y(:,1))
%ylim([-4 4])
grid on
title('Outputs')
ylabel('Position'), xlabel('Time')
legend('v','x')

subplot(2,1,2), plot((y(:,3))*180/pi)
legend('Theta')
grid on
% subplot(3,1,3), plot((y(:,4)))
% legend('w')
% grid on

% figure(1)
% plot(y(:,2))
% legend('v')
% grid on
% 
% figure(2)
% plot(y(:,1))
% legend('x')
% grid on
% 
% figure(3)
% plot((y(:,3))*180/pi)
% legend('teta')
% grid on
% 
% figure(4)
% plot(y(:,4))
% legend('w')
% grid on