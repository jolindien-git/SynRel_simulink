%INITSIMULINKOBS Initialise, simule et trace le modèle synRelSimuLutObs.slx.
%   Version autonome du modèle de simulation de la note (SynRel décrit par
%   cartographies FEMM, commande en courant, charge asservie en vitesse,
%   observateur de flux à gradient d'Orsolle-Tyberg et al. dans le
%   sous-système référencé fluxObserver.slx). Mêmes réglages que
%   ../noteSimulationSetup.m, sans dépendance aux autres dossiers : peut
%   être exporté pour une version antérieure de MATLAB (exportSimulinkObs).
%
%   Fichiers nécessaires dans le même dossier : synRelSimuLutObs.slx,
%   fluxObserver.slx, synRelSimuLutData.mat, setMotorParameters.m,
%   polePlacement2ndOrder.m, plotSimulationObs.m.
%
%   Variables créées :
%     motor      - paramètres moteur (setMotorParameters) + cartographies
%     trajectory - trajectoire de courants optimale
%     inverter, control, observer, math, Ts - réglages du modèle
%     scenario   - consignes et durée de simulation (modifiable)
%     out        - résultat de la simulation (out.logsout : signaux)
%
%   See also plotSimulationObs, exportSimulinkObs, setMotorParameters.

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

%% Onduleur et échantillonnage
Ts = 1e-4;                           % s - période de la commande
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
scenario.omegaRef      = 500 * pi / 30; % rad/s - 500 tr/min
scenario.loadMax       = 5;             % N.m - saturation du couple de charge asservi
scenario.loadTorque    = 1.7;           % N.m - couple de charge constant (si speedControl = 0)
scenario.omega0        = scenario.omegaRef;   % rad/s - vitesse initiale
scenario.theta0        = 0;             % rad - position initiale


%% Estimateur de flux
regressor.R         = motor.R;                    % Ohm
math.J = [0 -1; 1 0];                   % rotation de pi/2

%% Simulation
clear simulinkFolder commonFolder lutData lutName kElec kMeca
tic
out = sim('synRelSimuLutObs');
toc

%% tracé
fig = plotObs(out.logsout, "lambdadq_hat");
fig.set('Name', 'Regressor')

figure, 

subplot(211)
z = out.logsout.getElement("z").Values;
plot(z.Time, z.Data(:, 1))
legend('z')

subplot(212)
% lambda_ab_hat = out.logsout.getElement("lambda_ab_hat").Values;
% plot(lambda_ab_hat.Time, lambda_ab_hat.Data(:, 1))
% legend('lambda_a_hat')
lambda_ab_hat_0 = out.logsout.getElement("lambda_ab_hat_0").Values;
plot(lambda_ab_hat_0.Time, lambda_ab_hat_0.Data(:, 1))
legend('$\hat \lambda_\alpha 0$', 'Interpreter', 'latex')

