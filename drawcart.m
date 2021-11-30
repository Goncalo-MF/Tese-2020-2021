function drawcart(y,m,M,L)
x = y(1);
th = y(3);
Epot = m * 9.81 * L * cos(th-pi);
% kinematics
% x = cart position
% th = pendulum angle

% dimensions
W = 0.4*sqrt(M/7);  % cart width
H = 0.4*sqrt(M/7); % cart height
wr = .02; % wheel radius
mr = .05*sqrt(m); % mass radius

% positions
% y = wr/2; % cart vertical position
y = wr/2+H/2; % cart vertical position
w1x = x-.9*W/2;
w1y = 0;
w2x = x+.9*W/2-wr;
w2y = 0;

px = x + L*sin(th);
py = y - L*cos(th);

plot([-10 10],[0 0],'k','LineWidth',2)
hold on
rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[1 0.1 0.1])
rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])
rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])

plot([x px],[y py],'k','LineWidth',2) % fio

rectangle('Position',[px-mr/2,py-mr/2,mr,mr],'Curvature',1,'FaceColor',[.1 0.1 1]) % pend

th=th*180/pi;

h4=text(0.1, 0.1, ['phi: ', num2str(1)]);
h5=text(0.1, 0.17, ['Epot: ', num2str(1)]);
set(h4,'color','b', 'fontsize', 14);
set(h5,'color','b', 'fontsize', 14);
set(h4, 'string',['phi: ', num2str(th)]);
set(h5, 'string',['Epot: ', num2str(Epot)]);
  
xlim([-0.3 0.3]);
ylim([-0.25 0.25]);
set(gcf,'Position',[300 350 800 400])
drawnow
hold off