% =====================
% git: branch main
% =====================

simulinkFolder = fileparts(mfilename('fullpath'));
addpath(simulinkFolder);
commonFolder = fullfile(simulinkFolder, '..', '..', 'common');
if ~isfile(fullfile(simulinkFolder, 'setMotorParameters.m')) && isfolder(commonFolder)
    addpath(commonFolder);             % arborescence du projet (hors export)
end

%% Paramètres moteur, cartographies, trajectoire
motor = setMotorParameters("SynRelLaplace");
lutData = load(fullfile(simulinkFolder, 'synRelSimuLutData.mat'), 'motor', 'trajectory');
for lutName = ["lutInv", "lutTorque", "lutLinc"]
    motor.(lutName) = lutData.motor.(lutName);
end
trajectory = lutData.trajectory;
math.J = [0 -1; 1 0];                   % rotation de pi/2

%% Onduleur et échantillonnage
Ts = 1e-4;% / 10                            % s - période de la commande
inverter.vDC  = 300;                    % V - bus continu
inverter.vMax = inverter.vDC / 2;       % V - tension max

%% Boucles de courant (placement de pôles 2nd ordre, gains mis à l'échelle par L)
control.tauElec = 0.5e-3;               % s - constante de temps visée
control.zeta    = 0.7;                  % amortissement
kElec = polePlacement2ndOrder(control.zeta, 1 / control.tauElec);
control.kElecd  = kElec * motor.Ld;
control.kElecq  = kElec * motor.Lq;

%% Charge : PI sur le couple de charge asservissant la vitesse
control.tauMeca = 0.1;                  % s
kMeca = polePlacement2ndOrder(control.zeta, 1 / control.tauMeca);
control.kpSpeed = kMeca(1) * motor.J - motor.F;
control.kiSpeed = -kMeca(2) * motor.J;

%% Scénario de simulation (à modifier librement)
scenario = struct;
scenario.stopTime      = 3;             % s
scenario.tauRef        = 2;             % N.m - couple de référence
scenario.useTrajectory = 0;             % 1 : trajectoire optimale, 0 : courant constant
scenario.speedControl  = 1;             % 1 : charge asservie en vitesse
scenario.omegaRef      = 2*500 * pi / 30; % rad/s - 500 tr/min
scenario.loadMax       = 5;             % N.m - saturation du couple de charge asservi
scenario.loadTorque    = 1.7;           % N.m - couple de charge constant (si speedControl = 0)
scenario.omega0        = scenario.omegaRef;   % rad/s - vitesse initiale
scenario.theta0        = 0;             % rad - position initiale

%% Observateur de flux (nouveau)
observer.R         = motor.R;                    % Ohm
observer.harmonics = (-21:2:21);% [-7, -3, -1, 1, 5];
[observer.gamma, rho_theory] = find_optimal_gamma(observer.harmonics);
fprintf('Taux de convergence (theoritical) : %.4f rad^-1\n', rho_theory);

%% Simulation
clear simulinkFolder commonFolder lutData lutName kElec kMeca
tic
out = sim('synRelSimuLutObs');
toc

%% tracé
fig = plotObs(out.logsout, "lambdadqHat");
fig.set('Name', 'Observer (new)')

%% taux de convergence
lambdadq = out.logsout.getElement("lambdadq").Values.Data;
lambdadqHat = out.logsout.getElement("lambdadqHat").Values.Data;
theta_e = motor.p * out.logsout.getElement("theta").Values.Data;
try
    [rho_sim, diag] = estimate_decay_rate_from_sim(lambdadq, lambdadqHat, theta_e);
    fprintf('Taux de convergence identifie en simulation : %.4f rad^-1\n', rho_sim);
catch
    fprintf("estimate_decay_rate_from_sim: error \n")
end