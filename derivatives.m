function dy = derivatives(y,m,M,L,g,d,u)
% y = [x , ^x, o , ^o] states , y=xdot
% M massa do carro, m massa do pendulo, L tamanho do fio
% d fricçao, g grav no pendulo


Sy = sin(y(3));
Cy = cos(y(3));
D = m*L*L*(M+m*(1-Cy^2));

%derivadas 

dy(1,1) = y(2);
dy(2,1) = (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*(m*L*y(4)^2*Sy - d*y(2))) + m*L*L*(1/D)*u;
dy(3,1) = y(4);
dy(4,1) = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y(4)^2*Sy - d*y(2))) - m*L*Cy*(1/D)*u +.1*randn;


% -m*m*l*l*g*Sy*Sy
% m*l*l*m*l*y(4)^2*Sy
% %-m*l*l*d*y(2)
% %m*l*l*u
% 
% (m+M)*m*g*l*Sy
% m*l*Cy*m*l*y(4)^2*Sy
% %-m*l*Cy*d*y(2)
% %m*l*Cy*u
