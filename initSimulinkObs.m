% ===============
% git: branch delta_R 
% 
% Objectif: tester l'observation en présence d'erreur sur R
% scenario.omegaRef n'est plus constant
% ===============

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
Ts = 5e-5;                            % s - période de la commande
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

%% Scénario de simulation
scenario = struct;
scenario.stopTime      = 3;             % s
scenario.tauRef        = 2;             % N.m - couple de référence
scenario.useTrajectory = 0;             % 1 : trajectoire optimale, 0 : courant constant
scenario.speedControl  = 1;             % 1 : charge asservie en vitesse
scenario.loadMax       = 5;             % N.m - saturation du couple de charge asservi
scenario.loadTorque    = 1.7;           % N.m - couple de charge constant (si speedControl = 0)
scenario.theta0        = 0;             % rad - position initiale
scenario.omegaRef      = 500 * pi / 30; % rad/s - 500 tr/min

%-- Modulated speed reference trajectory
t_vec = (0:Ts:scenario.stopTime).';
epsilon = 0.05*2;          
nu      = .125/2;          % la note conseille 3/8 MAIS le contrôle a du mal à suivre
%-- ondulation
omegaPsi = nu * motor.p * scenario.omegaRef * t_vec;
omegaPsi(t_vec>2) = pi/2;
omegaRipples = scenario.omegaRef * epsilon * cos(omegaPsi);
scenario.omegaPsi = [t_vec, omegaPsi];
scenario.omegaRipples = [t_vec, omegaRipples];

%% Observateur de flux (nouveau)
observer.R         = 1.1 * motor.R;              % Ohm
observer.harmonics = -7:2:5; %-21:2:21;%
[observer.gamma, rho_theory] = find_optimal_gamma(observer.harmonics);
fprintf('Taux de convergence (theoritical) : %.4f rad^-1\n', rho_theory);

%% Simulation
clear simulinkFolder commonFolder lutData lutName kElec kMeca
modelName = 'synRelSimuLutObs';
% if bdIsLoaded(modelName)
%     open_system(modelName);
% end
tic
out = sim(modelName);
toc


%% tracé

fig = plotObs(out.logsout, "lambdadqHat");
fig.set('Name', 'Observer (extended)')
    
figure,
subplot(311)
deltaRHat = out.logsout.getElement("deltaRHat").Values;
plot(deltaRHat.Time, (motor.R - observer.R) * ones(size(deltaRHat.Time)), ...
     deltaRHat.Time, deltaRHat.Data); 
legend("delta R", "delta R (estim)")
subplot(312)
om = out.logsout.getElement("om").Values;
plot(om.Time, om.Data, ...
    scenario.omegaRipples(:,1), scenario.omegaRef + scenario.omegaRipples(:,2))
legend("w", "w ref")
subplot(313)
tau = out.logsout.getElement("tau").Values;
plot(tau.Time, tau.Data)
legend("tau")


