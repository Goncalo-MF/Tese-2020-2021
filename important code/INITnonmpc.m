clear all, close all, clc

M=0.08;
m=0.01;
mCart = M;  % pendulum mass
mPend = m;  % cart mass
g = -9.8;   % gravity of earth
L = 0.06;  % pendulum length
Kd = 1;    % cart damping
d= 1; %damping/fricçao
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

D1 = 0;

sys=ss(A,B,C,D1);

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

%%  Non Linear MPC
 
% y0=[0;0;0;0]; % posiçao inicial também no gráfico do MPC, entra no integral depois das derivadas

nx = 4;
ny = 2;
nu = 1;
nlobj = nlmpc(nx, ny, nu);

Ts = 0.1;
nlobj.Ts = Ts;

% nlobj.PredictionHorizon = 10;
% nlobj.ControlHorizon = 2;

% nlobj.PredictionHorizon = 20;
% nlobj.ControlHorizon = 15;

nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 4;

nlobj.Model.StateFcn = "penddisc";
nlobj.Model.IsContinuousTime = false;

nlobj.Model.NumberOfParameters = 1;

nlobj.Model.OutputFcn = 'pendout';

nlobj.Model.OutputFcn = @(x,u,Ts) [x(1); x(3)];

% opcional, o mpc faz sozinho mas demora mais nesse caso
nlobj.Jacobian.OutputFcn = @(x,u,Ts) [1 0 0 0; 0 0 1 0];

% %pretty good results
% 
% % standard mpc weights
% nlobj.Weights.OutputVariables = [3 3];
% nlobj.Weights.ManipulatedVariablesRate = 0.1;
% 
% %limites de x
% nlobj.OV(1).Max = 0.3;
% nlobj.OV(1).Min = -0.3;
% 
% nlobj.States(2).Max = 1.3;
% nlobj.States(2).Min = -1.3;
% nlobj.States(3).Max = 8;
% nlobj.States(3).Min = 8;
% nlobj.States(4).Max = 10000; % teoricamente não preciso de limites no w
% nlobj.States(4).Min = -10000; % teoricamente não preciso de limites no w
% 
% % limites de u (mv/output)
% nlobj.MV.Max = 1;
% nlobj.MV.Min = -1;

% standard mpc weights
nlobj.Weights.OutputVariables = [10 2];
nlobj.Weights.ManipulatedVariablesRate = 0.1;

%limites de x
nlobj.OV(1).Max = 0.12;
nlobj.OV(1).Min = -0.12;

nlobj.States(2).Max = 0.8;
nlobj.States(2).Min = -0.8;
nlobj.States(3).Max = 11;
nlobj.States(3).Min = -11;
nlobj.States(4).Max = 1000; % teoricamente não preciso de limites no w
nlobj.States(4).Min = -1000; % teoricamente não preciso de limites no w

% limites de u (mv/output)
nlobj.MV.Max = 0.8;
nlobj.MV.Min = -0.8;

x0 = [0.1;0.2;-pi/2;0.3];
u0 = 0.2;
validateFcns(nlobj, x0, u0, [], {Ts},[0 0]);

%% State Simulation

EKF = extendedKalmanFilter(@pendstate, @pendobs);

% initial state
x = [0;0;pi;0];
y = [x(1);x(3)];
EKF.State = x;

% force applied to the cart is zero at the beginning.
mv = 0;

%first desired position and angle (upright)
yref1 = [0 pi];

%second desired position and angle (upright but moved ot the right)
yref2 = [0 0];
yref3 = [0 pi];
yref4 = [0 0];
%to compute optimal control moves at each control interval
nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};




%% parte que dava mal, atualizar refs

% % Run the simulation for |20| seconds.
% Duration = 60
% % hbar = waitbar(0,'Simulation Progress');
% xHistory = x;
% mvHistory=mv;
% 
% for ct = 1:(60/Ts)
%     % Set references 
% %     if ct*Ts<10
% %         yref = yref1;
% %     elseif ct*Ts<20
% %         yref = yref2;
% %     elseif ct*Ts<30
% %         yref = yref3;
% %     elseif ct*Ts<40
% %         yref = yref4;
% %     elseif ct*Ts<50
% %         yref = yref3;
% %     else
% %         yref = yref4;
% %     end
%     yref=yref1;
%     % Correct previous prediction using current measurement 
%     xk = correct(EKF, y);
%     % Compute optimal control moves 
%     [mv,nloptions,info] = nlmpcmove(nlobj,xk,mv,yref,[],nloptions);
%     % Predict prediction model states for the next iteration
%     predict(EKF, [mv; Ts]);
%     % Implement first optimal control move and update plant states.
%     x = penddisc(x,mv,Ts);
%         % Generate sensor data with some white noise
%     y = x([1 3])+ randn(2,1)*0.001; 
%     % Save plant states for display.
%     mvHistory=[mvHistory mv];
%     xHistory = [xHistory x]; 
% %     waitbar(ct*Ts/20,hbar);
% end
% % close(hbar);
% 
% % % Plot the closed-loop response.
% figure
% subplot(2,2,1)
% plot(0:Ts:Duration,xHistory(1,:))
% xlabel('time')
% ylabel('z')
% title('cart position')
% subplot(2,2,2)
% plot(0:Ts:Duration,xHistory(2,:))
% xlabel('time')
% ylabel('zdot')
% title('cart velocity')
% subplot(2,2,3)
% plot(0:Ts:Duration,xHistory(3,:))
% xlabel('time')
% ylabel('theta')
% title('pendulum angle')
% subplot(2,2,4)
% plot(0:Ts:Duration,xHistory(4,:))
% xlabel('time')
% ylabel('thetadot')
% title('pendulum velocity')
% 
% o=3
% return;

%% simulink ( igual a initfcn basicamente)

mdl=0;
mdl = 'mpcsimulink';
open_system(mdl)
createParameterBus(nlobj,[mdl '/Nonlinear MPC Controller'],'BusNMPC',{Ts});
sim(mdl)

o=2
return;
