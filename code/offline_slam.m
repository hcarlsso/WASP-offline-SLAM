init

% True sensor positions 
m0 = [0 1 1 1 1 0 1 -1 0 -1 -1 -1 -1 0 -1 1]'*70;

% initial target positions
p0 = [0.9 -0.4]';

% Number of sensors 
M = length(m0)/2;

% sensor model
smod = exsensor('toa', M);

% Initial state
smod.x0 = p0;

% Sensor mod
smod.th = m0(:);

% Define pertubed network for later use
sigmath = 0.1;

% pertubed sensor positions 
m = m0 + sigmath * randn(2*M,1);

% Covariance of sensor positions
Pmm = sigmath^2 * eye(2*M);

% number of measurements
N = 8;
fs = 1;

% measurements
R = 1e-3 * eye(M);

% Motion model
mmod = exmotion('ctpva2d', fs);

% Total model
tmod = nl(mmod.f, smod.h, [6 0 M 2*M]);
tmod.fs = fs;
% Initial state
tmod.x0 = [p0' 2*pi/N pi/2 -0.02 2*pi/N]';
tmod.xlabel = mmod.xlabel;
tmod.th = m;
tmod.pe = R;

% Simulate four laps
y = simulate(tmod, 4*N);

figure(1);
xplot2(y)
hold on;
plot(smod, 'linewidth', 2 , 'fontsize',18)
% axis([-1.2 1.2 -1.2 1.2])

% Create estimation model 
ekfmod = tmod;

% Initial state
ekfmod.x0 = [p0' 0 0 0 0]';
% ekfmod.x0 = y.x(1,:);

% Known initial position for SLAM observation
ekfmod.px0 = 1000*diag([0 0 1 1 1 1]);
ekfmod.pv = 1e-3*diag([0 0 0 0 1 1]);

% True positions
ekfmod.th = m0;

% EKF for true sensor positions
xhat1 = ekf(ekfmod,y);

% EKF for perturbed positions
ekfmod.th = m;
ekfmod.P = Pmm;
xhat2 = ekf(ekfmod,y);

figure(2);
xplot2(xhat1, xhat2, 'conf', 90,'linewidth', 2, 'fontsize', 18)
hold on;
plot(smod, 'linewidth', 2 , 'fontsize',18)


%% Parameter estimation approach 

% Create estimation model 
nlsmod = tmod;

% Initial state
nlsmod.x0 = [p0' 0 0 0 0]';
% nlsmod.x0 = y.x(1,:);

% Known initial position for SLAM observation
nlsmod.px0 = 1000*diag([0 0 1 1 1 1]);
nlsmod.pv = 1e-3*diag([0 0 0 0 1 1]);

nlsmod.th = m;
nlsmod.P = Pmm;

modhat = estimate(nlsmod,y,...
    'x0mask',[0 0 1 1 1 1],...
    'ctol', 1e-2,...
    'gtol', 1e-2,...
    'disp', 'on');

figure(3);

hold on;
plot(smod, 'linewidth', 2 , 'fontsize',18)


