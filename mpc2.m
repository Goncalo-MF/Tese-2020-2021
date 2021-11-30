clear all, close all, clc

% nx=4;
% nu=1;
% ny=2;
% nlobj = nlmpc(nx, ny, nu);
% 
% ts=0.1;


m = 1;
M = 3;
L = 1;
g = -10;
d = 1; %damping/fricçao
%syms m M L g d

s = 1; % pendulum up (s=1)

syms x phi xdot phi phidot F
y=[ x ; xdot ; phi ; phidot ];


Sy = sin(y(3));
Cy = cos(y(3));
D = m*L*L*(M+m*(1-Cy^2));

%%
    
sys2(1)=  y(2);
sys2(2)= (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*(m*L*y(4)^2*Sy - d*y(2))) + m*L*L*(1/D)*F;
sys2(3)= y(4);
sys2(4)= (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y(4)^2*Sy - d*y(2))) - m*L*Cy*(1/D)*F +.1*randn;

sys=ss(sys2)