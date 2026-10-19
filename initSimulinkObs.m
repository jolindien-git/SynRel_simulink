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
Ts = 1e-4 / 10                            % s - période de la commande
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
scenario.useTrajectory = 1;             % 1 : trajectoire optimale, 0 : courant constant
scenario.speedControl  = 1;             % 1 : charge asservie en vitesse
scenario.SpeedModulate = 0;             % 1: modulation pour estimation de R
scenario.omegaRef      = 500 * pi / 30; % rad/s - 500 tr/min
scenario.loadMax       = 5;             % N.m - saturation du couple de charge asservi
scenario.loadTorque    = 1.7;           % N.m - couple de charge constant (si speedControl = 0)
scenario.omega0        = scenario.omegaRef;   % rad/s - vitesse initiale
scenario.theta0        = 0;             % rad - position initiale

% Modulated speed reference trajectory
t_vec = (0:Ts:scenario.stopTime).';
if scenario.SpeedModulate == 1
    epsilon = 0.05;          % Modulation amplitude (5%)
    nu      = 0.25;          % Non-integer harmonic order
    theta_e_vec = scenario.theta0 * motor.p + motor.p * scenario.omegaRef * t_vec;
    scenario.omegaRef_mat = [t_vec,...
        scenario.omegaRef * (1.0 + epsilon * cos(nu * theta_e_vec))];
else
    scenario.omegaRef_mat = [t_vec,...
        scenario.omegaRef * ones(size(t_vec))];
end

%% Observateur de flux (Orsolle-Tyberg et al., 2023)
if scenario.SpeedModulate == 1
    observer.R         = 1.1 * motor.R;                    % Ohm
else
    observer.R         = motor.R;                    % Ohm
end
observer.Ln        = (motor.Ld + motor.Lq) / 2;  % H - inductance nominale
observer.gamma     = 300;                        % gain d'adaptation
observer.lambda0   = [0 0];                      % Wb - flux alphabeta initial (faux)
observer.lambdaEq0 = 0.05;                       % Wb - flux équivalent initial (non nul)
observer.omegaMin  = 1;                          % rad/s - borne de |omega| pour d/dtheta
math.J = [0 -1; 1 0];                   % rotation de pi/2

%% Observateur de flux (nouveau)
observer_new.R = observer.R;
observer_new.harmonics = (-21:2:21);% [-7, -3, -1, 1, 5];
[observer_new.gamma, rho_theory] = find_optimal_gamma(observer_new.harmonics);
observer_new.gamma_R = 1; % 3
fprintf('Taux de convergence (theoritical) : %.4f rad^-1\n', rho_theory);

%% Simulation
clear simulinkFolder commonFolder lutData lutName kElec kMeca
tic
out = sim('synRelSimuLutObs');
toc

%% tracé

fig = plotObs(out.logsout, "lambdadqHat");
fig.set('Name', 'Observer (Pauline)')
    
fig = plotObs(out.logsout, "lambdadq_hat");
fig.set('Name', 'Observer (new)')

if scenario.SpeedModulate == 1
    fig = plotObs(out.logsout, "lambdadq_hat_extended");
    fig.set('Name', 'Observer (extended)')
    
    figure,
    subplot(211)
    delta_R = out.logsout.getElement("delta_R_extended").Values;
    plot(delta_R.Time, delta_R.Data, delta_R.Time, (motor.R - observer_new.R) * ones(size(delta_R.Time))); 
    legend("delta R (estim)", "R - obsR")
    subplot(212)
    om = out.logsout.getElement("om").Values;
    plot(om.Time, om.Data, scenario.omegaRef_mat(:,1), scenario.omegaRef_mat(:,2))
    legend("w", "w ref")
end

%% taux de convergence
lambdadq = out.logsout.getElement("lambdadq").Values.Data;
lambdadq_hat = out.logsout.getElement("lambdadq_hat").Values.Data;
theta_e = motor.p * out.logsout.getElement("theta").Values.Data;

[rho_sim, diag] = estimate_decay_rate_from_sim(lambdadq, lambdadq_hat, theta_e);
fprintf('Taux de convergence identifie en simulation : %.4f rad^-1\n', rho_sim);
