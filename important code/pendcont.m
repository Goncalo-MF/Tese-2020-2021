function dxdt = pendcont(x, u)
%% Continuous-time nonlinear dynamic model of a pendulum on a cart

%% parameters
M=0.08;
m=0.01;
mCart = M;  % pendulum mass
mPend = m;  % cart mass
g = -9.8;   % gravity of earth
L = 0.06;    % pendulum length
Kd = 1;    % cart damping
d= 1;


%A GRAVIDADE FOI MUDADA PARA POSITIVO E FAZ GRANDE DIFERENÇA
%A GRAVIDADE FOI MUDADA PARA POSITIVO E FAZ GRANDE DIFERENÇA
%A GRAVIDADE FOI MUDADA PARA POSITIVO E FAZ GRANDE DIFERENÇA
%A GRAVIDADE FOI MUDADA PARA POSITIVO E FAZ GRANDE DIFERENÇA


Sy = sin(x(3));
Cy = cos(x(3));
D = m*L*L*(M+m*(1-Cy^2));

% K =1.0e+04 *[-1.0000   -0.4747    1.0993    0.1571];
% newx = [0; 0; pi; 0];


% % setpoints
% % newx = [2; 0; pi; 0];
%  newx = [0; 0; pi; 0];
% 
% % if(t>12)
% %     newx = [-1; 0; pi; 0]; 
% % end 
% 
% % open loop (no control)
% % if(t>29)
% %   u = 0;
% % else
%                                             % && u ~= 0
% if (( x(3) > (pi-pi/5) ) && ( x(3) < (pi+pi/5)) || ...
%    ( x(3) > (-pi-pi/5) ) && ( x(3) < (-pi+pi/5)) )  
%     u = K * (newx - [x(1);x(2);x(3);x(4)]); % lqr control u = k*(newx-x)
% else
%     % energy based swing-up control % 1996 furata and astrom eq.6
%     % E=2/3*m2*l^2*x(4)^2 - m2*g*l*(cos(x(3))-1);
%     % u=0.2*m1*E*x(4)*cos(x(3));
% %         k=1;
% %         Ed= -m*g*L*cos(pi);
% %         E= m*L^2*y(4) - m*g*L*cos(y(3));
% %         u=k*y(4)*(E-Ed);
%      E= m*g*L*(cos(x(3)));
%      u=1*M*E*x(4)*(cos(x(3))); %velocidade com que ganha balanço
% %     end
% end

% %% Compute dxdt = dy
dxdt = x;
% v
dxdt(1) = x(2);
% ac
dxdt(2) = (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*(m*L*x(4)^2*Sy - d*x(2))) + m*L*L*(1/D)*u;
% w
dxdt(3) = x(4);
% ap
dxdt(4) = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*x(4)^2*Sy - d*x(2))) - m*L*Cy*(1/D)*u;

% % x
% z_dot = x(2);
% theta = x(3);
% theta_dot = x(4);
% % u
% F = u;
% % y
% %y = x;

% dxdt = x;
% % z_dot
% dxdt(1) = z_dot;
% % z_dot_dot
% dxdt(2) = (F - Kd*z_dot - mPend*L*theta_dot^2*sin(theta) + mPend*g*sin(theta)*cos(theta)) / (mCart + mPend*sin(theta)^2);
% % theta_dot
% dxdt(3) = theta_dot;
% % theta_dot_dot
% dxdt(4) = ((F - Kd*z_dot - mPend*L*theta_dot^2*sin(theta))*cos(theta)/(mCart + mPend) + g*sin(theta)) / (L - mPend*L*cos(theta)^2/(mCart + mPend));
