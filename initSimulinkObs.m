% =====================
% git: branch bench_data
%
% Objectifs: 
%   tester l'observateur de flux sur des données du banc
%   évaluer l'amélioration quand offset de u-Ri est estimé 
%       amélioration importante pour problem=3 (mesures HBK) et n_repeat >= 3
%       A VOIR: pas de facteur d'oubli dans estim offset -> problème si cet offset change 
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
n_repeat = 3;
[hbk, dspace] = deal(repeat_data(hbk, n_repeat), repeat_data(dspace, n_repeat));

problem = 1; % le problem 3 (à partir des mesures HBK qui est le + amélioré par offset: c'est là où il est le plus grand)
if problem == 1
    fprintf("*** PROBLEM: Estimate HBK flux from DSPACE signals \n")
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
    %-- outputs (only used for comparison)
    idq = timeseries([dspace.id, dspace.iq], t_vec);
    psidq = timeseries([hbk.psid, hbk.psiq], t_vec);
    fprintf("Mean speed %.2f [rpm] \n", mean(inputs.speed.Data) / pi * 30)
    
elseif problem == 2
    fprintf("*** PROBLEM: Estimate DSPACE flux from DSPACE signals \n")
    %-- inputs of observer = iab, vab, theta, speed
    inputs = struct;
    t_vec = dspace.t - dspace.t(1);
    inputs.iab = timeseries([dspace.ia, dspace.ib], ...
                            t_vec);
    inputs.vab = timeseries([dspace.va, dspace.vb], ...
                            t_vec);
    inputs.thetaSynRel = timeseries(unwrap(dspace.thetaSynrel), ...
                                    t_vec);
    speed = diff(inputs.thetaSynRel.Data) ./ diff(inputs.thetaSynRel.Time);
    inputs.speed = timeseries([speed(1); speed], ...
                              t_vec);
    %-- outputs (only used for comparison)
    idq = timeseries([dspace.id, dspace.iq], t_vec);
    psidq = timeseries([dspace.psid, dspace.psiq], t_vec);
    fprintf("Mean speed %.2f [rpm] \n", mean(inputs.speed.Data) / pi * 30)
else
    fprintf("*** PROBLEM: Estimate HBK flux from HBK measurements \n")
    %-- inputs of observer = iab, vab, theta, speed
    inputs = struct;
    t_vec = hbk.t - hbk.t(1);
    inputs.iab = timeseries([hbk.ia, hbk.ib], ...
                            t_vec);
    inputs.vab = timeseries([hbk.va, hbk.vb], ...
                            t_vec);
    inputs.thetaSynRel = timeseries(unwrap(dspace.thetaSynrel), ... % (thetaSynrel only available in DSPACE)
                                    t_vec);
    speed = diff(inputs.thetaSynRel.Data) ./ diff(inputs.thetaSynRel.Time);
    inputs.speed = timeseries([speed(1); speed], ...
                              t_vec);
    %-- outputs (only used for comparison)
    idq = timeseries([hbk.id, hbk.iq], t_vec);
    psidq = timeseries([hbk.psid, hbk.psiq], t_vec);
    fprintf("Mean speed %.2f [rpm] \n", mean(inputs.speed.Data) / pi * 30)
end


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