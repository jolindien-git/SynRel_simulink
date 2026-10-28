function fig = plotObs(logsout, name_idq, name_lambdadq, name_lambdadqHat)

idq = logsout.getElement(name_idq).Values;
lambdadq = logsout.getElement(name_lambdadq).Values;
lambdadqHat = logsout.getElement(name_lambdadqHat).Values;
lambda_err = lambdadq.Data - lambdadqHat.Data;

fig = figure;

tWindows = {[0, idq.Time(end)], [idq.Time(end) - .1, idq.Time(end)]};
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
        lambda_err_norm = vecnorm(lambda_err, 2, 2);
        semilogy(lambdadq.Time, lambda_err_norm)
        legend('$|\lambda_{dq} - \hat \lambda_{dq}|$', 'Interpreter', 'latex');
        ylim([1e-6, 1])
    else
        plot(lambdadq.Time, lambda_err)
        legend('$\lambda_{dq} - \hat \lambda_{dq}$', 'Interpreter', 'latex');
    end
    xlim(tWindow); grid on

    linkaxes([ax1, ax2, ax3], 'x');
end