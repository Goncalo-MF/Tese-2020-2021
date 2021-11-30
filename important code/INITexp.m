clear all, close all, clc

m = 0.25;
M = 0.32;
L = 0.07;
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

y0=[0;0;0.1;0];

load simoutfile.mat;