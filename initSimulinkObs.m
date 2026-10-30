% =====================
% git: branch bench_data
%
% Objectif: tester l'observateur de flux sur des données du banc
% =====================
clearvars; close all;

simulinkFolder = fileparts(mfilename('fullpath'));
addpath(simulinkFolder);
commonFolder = fullfile(simulinkFolder, '..', '..', 'common');
if ~isfile(fullfile(simulinkFolder, 'setMotorParameters.m')) && isfolder(commonFolder)
    addpath(commonFolder);             % arborescence du projet (hors export)
end

%% load data
%-- read file
record = load('record_500rpm_id5_iq5.mat').record;
[hbk, dspace] = deal(record.hbk, record.dspace);

%-- repeat signals (longer trajectories)
n_repeat = 0;
[hbk, dspace] = deal(repeat_data(hbk, n_repeat), repeat_data(dspace, n_repeat));

%-- correction: estimate hbk voltage from dspace signals
delta_v_corr = @(i) (9.8 + .121 * abs(i)) .* i ./ sqrt(i.^2 + .652^2);
v_corr = @(u_dspace, i) u_dspace - delta_v_corr(i);

%-- inputs of observer = iab, vab (corrected), theta, speed
inputs = struct;
t_vec = dspace.t - dspace.t(1);
inputs.iab = timeseries([dspace.ia, dspace.ib], ...
                        t_vec);
inputs.vab = timeseries([v_corr(dspace.va, dspace.ia), v_corr(dspace.vb, dspace.ib)], ...
                        t_vec);
inputs.vab.Data(2:end) = inputs.vab.Data(1:end-1); % delayed by one step
inputs.thetaSynRel = timeseries(unwrap(dspace.thetaSynrel), ...
                                t_vec);
speed = diff(inputs.thetaSynRel.Data) ./ diff(inputs.thetaSynRel.Time);
inputs.speed = timeseries([speed(1); speed], ...
                          t_vec);
%--
idq = timeseries([dspace.id, dspace.iq], t_vec);
psidq = timeseries([hbk.psid, hbk.psiq], t_vec);
fprintf("Mean speed %.2f [rpm] \n", mean(inputs.speed.Data) / pi * 30)

%% motor
motor = setMotorParameters();

%% Config Simulink
Ts = 5e-5;                               % s - période simulink
scenario = struct;
scenario.stopTime      = t_vec(end);     % s

%% Observateur de flux
observer.R         = motor.R;                    % Ohm
observer.harmonics =  (-7:2:9);
[observer.gamma, rho_theory] = find_optimal_gamma(observer.harmonics);

%% Simulation
clear simulinkFolder commonFolder lutData lutName kElec kMeca
modelName = 'synRelSimuLutObs';
% if bdIsLoaded(modelName)
%     open_system(modelName);
% end
tic
out = sim(modelName);
toc

%% tracés

%-- read Simulink outputs
idq = out.logsout.getElement("idq").Values;
lambdadq = out.logsout.getElement("lambdadq").Values;
lambdadqHat = out.logsout.getElement("lambdadqHat").Values;
lambdadqHat_offset = out.logsout.getElement("lambdadqHat_offset").Values;
offsetabHat = out.logsout.getElement("offsetabHat").Values;
theta_e = out.logsout.getElement("thetaElec").Values;
theta_m = timeseries(theta_e.Data / motor.p, theta_e.Time);

%-- plot observer results
fprintf("\n ========= without offset ======== \n")
fig = plotObs(theta_m, idq, lambdadq, lambdadqHat);
fig.set('Name', 'Observer')

fprintf("\n ========= with offset ======== \n")
fig = plotObs(theta_m, idq, lambdadq, lambdadqHat_offset);
fig.set('Name', 'Observer (offset)')

%-- offset estimate convergence
offset_ab = mean(inputs.vab - motor.R * inputs.iab);
figure
subplot(211)
plot(offsetabHat.Time, offset_ab(1) * ones(numel(offsetabHat.Time), 1), '--', ...
     offsetabHat.Time, offsetabHat.Data(:, 1))
legend('mean$(v_\alpha - R i_\alpha)$', 'estim', 'Interpreter', 'latex')
subplot(212)
plot(offsetabHat.Time, offset_ab(2) * ones(numel(offsetabHat.Time), 1), '--', ...
     offsetabHat.Time, offsetabHat.Data(:, 2))
legend('mean$(v_\beta - R i_\beta)$', 'estim', 'Interpreter', 'latex')


%% taux de convergence
fprintf('Taux de convergence (theorique) : %.4f rad^-1\n', rho_theory);

try
    [rho_sim, diag] = estimate_decay_rate_from_sim(lambdadq.Data, lambdadqHat.Data, theta_e.Data);
    fprintf('Taux de convergence identifie en simulation : %.4f rad^-1\n', rho_sim);
catch
    fprintf("estimate_decay_rate_from_sim: error \n")
end