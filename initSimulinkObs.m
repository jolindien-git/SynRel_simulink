% =====================
% git: branch bench_data
%
% Objectif: tester l'observateur de flux sur des données du banc
% =====================
clearvars; close all; clc;

simulinkFolder = fileparts(mfilename('fullpath'));
addpath(simulinkFolder);
commonFolder = fullfile(simulinkFolder, '..', '..', 'common');
if ~isfile(fullfile(simulinkFolder, 'setMotorParameters.m')) && isfolder(commonFolder)
    addpath(commonFolder);             % arborescence du projet (hors export)
end

%% motor
motor = setMotorParameters();

%% data
record = load('record_500rpm_id5_iq5.mat').record;
[dspace, hbk] = deal(record.dspace, record.hbk);

t0 = max(hbk.t(1), dspace.t(1));
tf = min(hbk.t(end), dspace.t(end)) - t0;

hbk_vdq = timeseries([hbk.vd, hbk.vq], hbk.t - t0);
hbk_idq = timeseries([hbk.id, hbk.iq], hbk.t - t0);
hbk_psidq = timeseries([hbk.psid, hbk.psiq], hbk.t - t0);

dspace_vdq = timeseries([dspace.vd, dspace.vq], dspace.t - t0);
dspace_idq = timeseries([dspace.id, dspace.iq], dspace.t - t0);
dspace_psidq = timeseries([dspace.psid, dspace.psiq], dspace.t - t0);

thetaSynRel = timeseries(dspace.thetaSynrel, dspace.t - t0);
dth = diff(dspace.thetaSynrel);
thetaSynRel.Data(2:end) = cumtrapz(mod(dth, 2*pi));%%%%%%%
speed = mod(dth, 2*pi) ./ diff(dspace.t);
speed_mean = mean(speed);
fprintf("Mean speed %.2f [rpm] \n", speed_mean / pi * 30)

%% Scénario de simulation (à modifier librement)
Ts = 1e-5                            % s - période de la commande
scenario = struct;
scenario.stopTime      = tf;             % s

%% Observateur de flux (nouveau)
observer.R         = motor.R;                    % Ohm
observer.harmonics = (-7:2:9);% [-7, -3, -1, 1, 5];
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
fig = plotObs(out.logsout, "dspace_idq", "dspace_lambdadq", "dspace_lambdadqHat");
fig.set('Name', 'Observer (dspace)')
fig = plotObs(out.logsout, "hbk_idq", "hbk_lambdadq", "hbk_lambdadqHat");
fig.set('Name', 'Observer (hbk)')

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