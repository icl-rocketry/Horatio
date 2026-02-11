function [z_posterior, P_posterior] = compute_posterior(K, C, R, z_prior, P_prior, y_measured)
% computes posterior values
    
    % calculate expected measurement and the deviation
    y_expected = C * z_prior;
    innovation = y_measured - y_expected;

    % calculate posterior augmented state
    z_posterior = z_prior + K * innovation;
    
    % calculate covariance matrix
    KC = K * C;
    I = eye(size(KC));
    P_posterior = (I - KC) * P_prior * transpose(I - KC) + K * R * transpose(K);
end