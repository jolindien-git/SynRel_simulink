function fig = plotSimulationObs(logsout, options)
%PLOTSIMULATIONOBS Trace les signaux enregistrés par synRelSimuLutObs.slx.
%   PLOTSIMULATIONOBS(logsout) trace les courants dq, le flux dq vrai et
%   estimé par l'observateur, l'erreur d'estimation et les couples (FEMM,
%   reconstruit par l'observateur, terme croisé) à partir du Dataset
%   logsout renvoyé par la simulation (out.logsout).
%
%   PLOTSIMULATIONOBS(logsout, tWindow=[t1 t2]) limite les tracés à une
%   fenêtre temporelle. fig = PLOTSIMULATIONOBS(...) renvoie la figure.
%
%   See also initSimulinkObs.

arguments
    logsout (1, 1) Simulink.SimulationData.Dataset
    options.tWindow (1, 2) double = [0 inf]
end

fig = figure('Name', 'Simulation SynRel avec observateur');
idq = getSignal(logsout, 'idq');
idqr = getSignal(logsout, 'idqr');
lambdadq = getSignal(logsout, 'lambdadq');
lambdadqHat = getSignal(logsout, 'lambdadqHat');
tau = getSignal(logsout, 'tau');
tauHat = getSignal(logsout, 'tauHat');
tauCrossHat = getSignal(logsout, 'tauCrossHat');

lambdadq_hat = getSignal(logsout, 'lambdadq_hat');
params_hat = getSignal(logsout, 'params_hat');
tau_hat_lin = getSignal(logsout, 'tau_hat_lin');

% Flux estimé rééchantillonné sur la base de temps du vrai flux
fluxHat = interp1(lambdadqHat.Time, lambdadqHat.Data, lambdadq.Time, 'previous', 'extrap');

subplot(4, 1, 1)
plot(idq.Time, idq.Data(:, 1), idqr.Time, idqr.Data(:, 1));
legend('$i_d$', '$i_d^r$', 'Interpreter', 'latex', 'Location', 'eastoutside');
xlim(options.tWindow); grid on

subplot(4, 1, 2)
plot(idq.Time, idq.Data(:, 2), idqr.Time, idqr.Data(:, 2));
legend('$i_q$', '$i_q^r$', 'Interpreter', 'latex', 'Location', 'eastoutside');
xlim(options.tWindow); grid on

subplot(4, 1, 3)
plot(lambdadq.Time, lambdadq.Data, lambdadq.Time, fluxHat, ...
    lambdadq_hat.Time, lambdadq_hat.Data, ...
    '--');
ylabel('$\lambda_{dq}$ (Wb)', 'Interpreter', 'latex');
legend('$\lambda_d$', '$\lambda_q$', '$\hat\lambda_d$', '$\hat\lambda_q$', ...
    'Interpreter', 'latex', 'Location', 'eastoutside');
xlim(options.tWindow); grid on

subplot(4, 1, 4)
plot(tau.Time, tau.Data, 'k', ...
    tauHat.Time, tauHat.Data, '--', ...
    tau_hat_lin.Time, tau_hat_lin.Data, ...
    tauCrossHat.Time, tauCrossHat.Data, ':');
ylabel('$\tau$ (N.m)', 'Interpreter', 'latex');
xlabel('Temps (s)');
legend('$\tau$ (FEMM)', '$\hat\tau$ (Pauline)', ...
    '$\hat\tau^{lin}$', '$\hat\tau_{cross}$', ...
    'Interpreter', 'latex');
xlim(options.tWindow); grid on


figure
subplot(3, 1, 1)
plot(lambdadq.Time, lambdadq.Data(:,1), lambdadq.Time, fluxHat(:,1), ...
    lambdadq_hat.Time, lambdadq_hat.Data(:,1), ...
    '--');
ylabel('$\lambda_{d}$ (Wb)', 'Interpreter', 'latex');
legend('$\lambda_d$', '$\hat\lambda_d$ (Pauline)', '$\hat\lambda_d$', ...
    'Interpreter', 'latex');
xlim(options.tWindow); grid on

subplot(3, 1, 2)
plot(lambdadq.Time, lambdadq.Data(:,2), lambdadq.Time, fluxHat(:,2), ...
    lambdadq_hat.Time, lambdadq_hat.Data(:,2), ...
    '--');
ylabel('$\lambda_{q}$ (Wb)', 'Interpreter', 'latex');
legend('$\lambda_q$', '$\hat\lambda_q$ (Pauline)', '$\hat\lambda_q$', ...
    'Interpreter', 'latex');
xlim(options.tWindow); grid on

subplot(3, 1, 3)
plot(lambdadq.Time, fluxHat - lambdadq.Data, ...
    lambdadq.Time, lambdadq_hat.Data - lambdadq.Data);
ylabel('$\hat\lambda_{dq} - \lambda_{dq}$ (Wb)', 'Interpreter', 'latex');
legend('$d$ (Pauline)', '$q$ (Pauline)', '$d$', '$q$', 'Interpreter', 'latex');
xlim(options.tWindow); grid on
rmse(lambdadq_hat.Data(end-1000,:), lambdadq.Data(end-1000,:))
rmse(fluxHat(end-1000,:), lambdadq.Data(end-1000,:))

% figure;
% plot(params_hat.Time, params_hat.Data)

end

function ts = getSignal(logsout, name)
%GETSIGNAL Série temporelle du signal nommé, données en N x largeur.
ts = logsout.getElement(name).Values;
data = ts.Data;
if ndims(data) == 3
    data = squeeze(data);
    if size(data, 1) ~= numel(ts.Time)
        data = data.';
    end
end
ts = struct('Time', ts.Time, 'Data', double(data));
end
