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
scenario.loadMax       = 5;             % N.m - saturation du couple de charge asservi
scenario.loadTorque    = 1.7;           % N.m - couple de charge constant (si speedControl = 0)
scenario.theta0        = 0;             % rad - position initiale
omegaRef_Nominal      = 2*500 * pi / 30; % rad/s - 500 tr/min

%-- Modulated speed reference trajectory
t_vec = (0:Ts:scenario.stopTime).';
epsilon = 0.05;          % Modulation amplitude (5%)
nu      = .125;          % Non-integer harmonic order
theta_e_vec = scenario.theta0 * motor.p + motor.p * omegaRef_Nominal * t_vec;
tf_ramp = 2;
ramp = min(omegaRef_Nominal, omegaRef_Nominal * t_vec / tf_ramp);
%-- ondulation
omegaRef = omegaRef_Nominal * (1.0 + epsilon * cos(nu * theta_e_vec));
%-- ramp
% omegaRef = ramp;
%-- ramp & ondulation
% omegaRef = ramp .* (1.0 + epsilon * cos(nu * theta_e_vec));

scenario.omegaRef = [t_vec, omegaRef];
scenario.omega0 = omegaRef(1);          % rad/s - vitesse initiale

%% Observateur de flux (nouveau)
observer.R         = 1.1 * motor.R;              % Ohm
observer.harmonics = (-21:2:21);% [-7, -3, -1, 1, 5];
[observer.gamma, rho_theory] = find_optimal_gamma(observer.harmonics);
fprintf('Taux de convergence (theoritical) : %.4f rad^-1\n', rho_theory);

%% Simulation
clear simulinkFolder commonFolder lutData lutName kElec kMeca
modelName = 'synRelSimuLutObs';
if bdIsLoaded(modelName)
    open_system(modelName);
end
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
plot(om.Time, om.Data, scenario.omegaRef(:,1), scenario.omegaRef(:,2))
legend("w", "w ref")
subplot(313)
tau = out.logsout.getElement("tau").Values;
plot(tau.Time, tau.Data)
legend("tau")


