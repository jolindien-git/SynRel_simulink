% lambdadq = out.logsout.getElement("lambdadq").Values.Data;
% lambdadqHat = out.logsout.getElement("lambdadq_hat").Values.Data;
% theta_e = motor.p * out.logsout.getElement("theta").Values.Data;
% 
% [rho_sim, diag] = estimate_decay_rate_from_sim(lambdadq, lambdadqHat, theta_e);
% fprintf('Taux identifie en simulation : %.4f rad^-1\n', rho_sim);

function [rho_est, diag] = estimate_decay_rate_from_sim(lambda_dq_true, lambda_dq_hat, theta_e, min_turns_after_peak, floor_margin_ratio)
% ESTIMATE_DECAY_RATE_FROM_SIM Estimate empirical decay rate rho (rad^-1) from trajectories.
%
% Inputs:
%   lambda_dq_true       : True flux linkage matrix [L x 2] (Wb)
%   lambda_dq_hat        : Estimated flux linkage matrix [L x 2] (Wb)
%   theta_e              : Electrical position vector [L x 1] (rad)
%   min_turns_after_peak : Electrical turns skipped after global peak (default: 1.0)
%   floor_margin_ratio   : Factor above steady-state floor (default: 3.0)
%
% Outputs:
%   rho_est              : Estimated decay rate (rad^-1)
%   diag                 : Diagnostic struct (envelope, valid indices, fit coefficients)

    if nargin < 4, min_turns_after_peak = 1.0; end
    if nargin < 5, floor_margin_ratio = 3.0; end

    theta = unwrap(theta_e(:));
    error_vec = lambda_dq_true - lambda_dq_hat;
    err_norm = sqrt(sum(error_vec.^2, 2));

    % 1. Local upper peaks detection
    [err_peaks, locs] = findpeaks(err_norm);
    if length(locs) < 4
        error('Insufficient local peaks to evaluate exponential envelope.');
    end
    theta_peaks = theta(locs);

    % 2. Peaking transient exclusion
    [~, max_idx] = max(err_peaks);
    theta_peak_max = theta_peaks(max_idx);
    theta_start = theta_peak_max + min_turns_after_peak * (2.0 * pi);

    % 3. Noise floor / unmodeled harmonic floor detection (last 20% samples)
    tail_len = max(floor(0.2 * length(err_peaks)), 5);
    noise_floor = median(err_peaks(end - tail_len + 1 : end));
    floor_threshold = noise_floor * floor_margin_ratio;

    % 4. Select linear regime window
    valid_mask = (theta_peaks >= theta_start) & (err_peaks > floor_threshold);
    if sum(valid_mask) < 3
        error('Could not isolate linear decay window. Check data duration or tuning.');
    end

    theta_fit = theta_peaks(valid_mask);
    log_err_fit = log(err_peaks(valid_mask));

    % 5. Linear fit on log-envelope
    p = polyfit(theta_fit, log_err_fit, 1);
    rho_est = -p(1);

    % Diagnostic outputs
    diag.rho_est = rho_est;
    diag.intercept = p(2);
    diag.theta_fit = theta_fit;
    diag.err_fit = err_peaks(valid_mask);
    diag.noise_floor = noise_floor;
end
