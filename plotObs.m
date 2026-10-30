function fig = plotObs(theta_m, idq, lambdadq, lambdadqHat)

lambda_err = timeseries(lambdadq.Data - lambdadqHat.Data, ...
                        lambdadq.Time);
%-- looking for last mechanical turn
idxs = find(theta_m.Data > theta_m.Data(end) - 2 * pi);
tWindow = [theta_m.Time(idxs(1)), theta_m.Time(end)];

fig = figure;
tWindows = {[0, idq.Time(end)], tWindow};
for k = 1:2
    tWindow = tWindows{k};
    ax1 = subplot(3, 2, 0 + k);
    plot(idq.Time, idq.Data(:, 1), idq.Time, idq.Data(:, 2))
    legend('$i_d$', '$i_q$', 'Interpreter', 'latex');
    xlim(tWindow); grid on
    
    ax2 = subplot(3, 2, 2 + k);
    plot(lambdadq.Time, lambdadq.Data(:, 1), lambdadq.Time, lambdadq.Data(:, 2), ...
        lambdadqHat.Time, lambdadqHat.Data(:, 1), lambdadqHat.Time, lambdadqHat.Data(:, 2))
    legend('$\lambda_d$', '$\lambda_q$', '$\hat\lambda_d$', '$\hat\lambda_q$', ...
        'Interpreter', 'latex');
    xlim(tWindow); grid on
    
    ax3 = subplot(3, 2, 4 + k);
    if k == 1
        lambda_err_norm = vecnorm(lambda_err.Data, 2, 2);
        semilogy(lambda_err.Time, lambda_err_norm)
        legend('$|\lambda_{dq} - \hat \lambda_{dq}|$', 'Interpreter', 'latex');
        ylim([1e-4, 1])
    else
        plot(lambda_err.Time, lambda_err.Data)
        legend('$\lambda_{dq} - \hat \lambda_{dq}$', 'Interpreter', 'latex');
    end
    xlim(tWindow); grid on

    linkaxes([ax1, ax2, ax3], 'x');
    
    %-- print lambda_err stats
    if k == 2
        fprintf("=== Time Window [%.2f s; %.2f s] \n", tWindow)
        idxs = find(lambda_err.Time > tWindow(1) & lambda_err.Time < tWindow(2));
        mean_lambda = mean(lambdadq.Data(idxs, :), 1);
        mean_lambdaHat = mean(lambdadqHat.Data(idxs, :), 1);
        mean_gap = mean_lambdaHat - mean_lambda;
        fprintf("Flux (measurement): mean d/q %.3f/%.3f\n", mean_lambda)
        fprintf("Flux (obs): mean d/q = %.3f/%.3f \n", mean_lambdaHat)
        fprintf("Mean gap d/q = %.2f/%.2f [mWb]\n", mean_gap*1000)
        rmse_ripple = rmse(lambdadq.Data(idxs, :) - mean(lambdadq.Data(idxs, :)), lambdadqHat.Data(idxs, :) - mean(lambdadqHat.Data(idxs, :)), 1);
        fprintf("Ripple rmse d/q = %.2f/%.2f [mWb]\n", rmse_ripple*1000)
    end
end