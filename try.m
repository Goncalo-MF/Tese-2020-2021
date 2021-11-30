%pkg load control 
clear all, close all, clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = 1; % pendulum up (s=1)


syms x xdot teta tetadot;
y=[x xdot teta tetadot];
Sy = sin(y(3));
Cy = cos(y(3));
D = m*L*L*(M+m*(1-Cy^2));

A = [0 1 0 0;
    0 -m*L*L*d/D -s*m*L*Cy*d/D 0;
    0 0 0 1;
    0 -s*/D -s*(m+M)*g/D m*L*Cy*m*l*Sy];

B = [0; m*L*L/D; 0; s*m*L*Cy/D];

% I=m*L*L/3
% D=(I*(M+m)+M*L*L*m)
% 
% A = [0 1 0 0;
%     0 -d*(I-m*L*L)/D -m*m*L*L*g/D 0;
%     0 0 0 1;
%     0 -s*d*m*L/D -s*(m+M)*m*L*g/D 0];
% 
% B = [0; (I+m*L*L)*d/D; 0; s*m*L/D];

Q = [1 0 0 0; %Q maior aumenta performance mas aumenta custo também
    0 1 0 0;  %Q para cada variavel, x xdot, o odot
    0 0 10 0; %maior penalizaçao nos tetas porque nao podem ter grandes margem de erro
    0 0 0 100];

R = .0001; % basicamente eletricidade de borla

%%
det(ctrb(A,B))
K = lqr(A,B,Q,R)

%%
K = lqr(A,B,Q,R);

tspan = 0:.001:20;
if(s==-1)
    y0 = [0; 0; 2; 0];
    [t,y] = ode45(@(t,y)derivatives(y,m,M,L,g,d,-K*(y-[0; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [0; 0; pi; 0];
%     [t,y] = ode45(@(t,y)((A-B*K)*(y-[3; 0; pi; 0])),tspan,y0,gh);
    [t,y] = ode45(@(t,y)derivatives(y,m,M,L,g,d,-K*(y-[3; 0; pi; 0] )),tspan,y0);
else
    
end

for k=1:100:length(t)
    drawcart(y(k,:),m,M,L);
end

subplot(2,1,1), plot(y(:,2))
hold on
plot(y(:,1))
%ylim([-4 4])
grid on
title('Variables')
ylabel('Saída'), xlabel('Amostra')

subplot(2,1,2), plot(y(:,3))
hold on
%plot(y(:,4))
grid on