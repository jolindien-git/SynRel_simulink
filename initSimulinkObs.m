% =====================
% git: branch bench_data
%
% Objectif: tester l'observateur de flux sur des données du banc
% =====================
clearvars; %close all; clc;

simulinkFolder = fileparts(mfilename('fullpath'));
addpath(simulinkFolder);
commonFolder = fullfile(simulinkFolder, '..', '..', 'common');
if ~isfile(fullfile(simulinkFolder, 'setMotorParameters.m')) && isfolder(commonFolder)
    addpath(commonFolder);             % arborescence du projet (hors export)
end

%% config


%% motor
motor = setMotorParameters();

%% data
record = load('record_500rpm_id5_iq5.mat').record;
% traj = record.dspace;
traj = record.hbk; 


t0 = traj.t(1);
tf = traj.t(end) - t0;
t_vec = traj.t - t0;
thetaSynRel = timeseries(unwrap(record.dspace.thetaSynrel), t_vec);
vab = timeseries([traj.va, traj.vb], t_vec);
iab = timeseries([traj.ia, traj.ib], t_vec);
idq = timeseries([traj.id, traj.iq], t_vec);
psidq = timeseries([traj.psid, traj.psiq], t_vec);

speed = diff(thetaSynRel.Data) ./ diff(traj.t);
speed = timeseries([speed(1); speed], t_vec);
speed_mean = mean(speed.Data);
fprintf("Mean speed %.2f [rpm] \n", speed_mean / pi * 30)

%-- centrer
% centrer = @(x) x - mean(x);
% Park = @(a, b, th) [a .* cos(th) + b .* sin(th), ...
%                        -a .* sin(th) + b .* cos(th)];
% vab.Data = [centrer(traj.va), centrer(traj.vb)];
% iab.Data = [centrer(traj.ia), centrer(traj.ib)];


%% Scénario de simulation (à modifier librement)
Ts = 1e-4                            % s - période de la commande
scenario = struct;
scenario.stopTime      = tf;             % s

%% Observateur de flux
observer.R         = motor.R;                    % Ohm
observer.harmonics = (-7:2:9);
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
fig = plotObs(out.logsout, "idq", "lambdadq", "lambdadqHat");
fig.set('Name', 'Observer')

%% taux de convergence
% lambdadq = out.logsout.getElement("lambdadq").Values.Data;
% lambdadqHat = out.logsout.getElement("lambdadqHat").Values.Data;
% theta_e = motor.p * out.logsout.getElement("theta").Values.Data;
% try
%     [rho_sim, diag] = estimate_decay_rate_from_sim(lambdadq, lambdadqHat, theta_e);
%     fprintf('Taux de convergence identifie en simulation : %.4f rad^-1\n', rho_sim);
% catch
%     fprintf("estimate_decay_rate_from_sim: error \n")
% end