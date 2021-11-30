function dy = derivatives(t,y,m,M,L,g,d,K)
% y = [x , ^x, o , ^o] states , y=xdot
% M massa do carro, m massa do pendulo, L tamanho do fio
% d fricçao, g grav no pendulo

Sy = sin(y(3));
Cy = cos(y(3));
D = m*L*L*(M+m*(1-Cy^2));


% setpoints
% newx = [2; 0; pi; 0];
newx = [0; 0; pi; 0];

% if(t>12)
%     newx = [-1; 0; pi; 0]; 
% end 

% open loop (no control)
if(t>29)
  u = 0;
else
                                            % && u ~= 0
    if (( y(3) > (pi-pi/5) ) && ( y(3) < (pi+pi/5)) || ...
       ( y(3) > (-pi-pi/5) ) && ( y(3) < (-pi+pi/5)) )  
        u = K * (newx - [y(1);y(2);y(3);y(4)]); % lqr control u = k*(newx-x)
    else
        % energy based swing-up control % 1996 furata and astrom eq.6
        % E=2/3*m2*l^2*x(4)^2 - m2*g*l*(cos(x(3))-1);
        % u=0.2*m1*E*x(4)*cos(x(3));
%         k=1;
%         Ed= -m*g*L*cos(pi);
%         E= m*L^2*y(4) - m*g*L*cos(y(3));
%         u=k*y(4)*(E-Ed);
         E= 0.85*m*g*L*(cos(pi-y(3)));
         u=0.2*M*E*y(4)*(cos(pi-y(3)));
    end
end


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
