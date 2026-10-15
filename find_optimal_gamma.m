% ks = 1:2:21;
% gammas = zeros(size(ks));
% rhos = zeros(size(ks));
% for i = 1:numel(ks)
%     k = ks(i);
%     harmonics = (-k:2:k);
%     [gamma_opt, rho_max] = find_optimal_gamma(harmonics);
%     sprintf("k %i  gamma %.3f  rho %.3f", k, gamma_opt, rho_max)
%     gammas(i) = gamma_opt;
%     rhos(i) = rho_max;
% end
% figure, plot(ks, gammas, ks, rhos); 
% legend("$\gamma^*$", "$\rho$", 'Interpreter', 'latex')
% xlabel("k")


% [gamma_opt, rho_max] = find_optimal_gamma([-7, -3, -1, 1, 5])


function [gamma_opt, rho_max] = find_optimal_gamma(harmonics, gamma_bounds)
% FIND_OPTIMAL_GAMMA Find adaptation gain maximizing exact decay rate rho.
%
% Inputs:
%   harmonics    : Vector of odd harmonic orders K (e.g., [-7, -3, -1, 1, 5])
%   gamma_bounds : 2-element vector [gamma_min, gamma_max] (optional)
%
% Outputs:
%   gamma_opt    : Optimal gain value
%   rho_max      : Maximum decay rate (rad^-1)

    if nargin < 2
        gamma_bounds = [0.01, 5.0];
    end

    harmonics = harmonics(:).';
    cost_function = @(gamma) -compute_spectral_decay_rate(gamma, harmonics);

    options = optimset('TolX', 1e-4, 'Display', 'off');
    [gamma_opt, neg_rho_max] = fminbnd(cost_function, gamma_bounds(1), gamma_bounds(2), options);
    rho_max = -neg_rho_max;
end

function rho = compute_spectral_decay_rate(gamma, harmonics)
    % A_gamma eigenvalue calculation
    k0 = [0, harmonics];
    n = length(k0);

    g = ones(n, 1);
    g(1) = -1.0;

    D = diag(1j * double(k0));
    A_gamma = D - gamma * (g * g.'); % Non-conjugate transpose

    eigenvalues = eig(A_gamma);
    rho = -max(real(eigenvalues));
end