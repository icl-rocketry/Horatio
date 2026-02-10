function [z_posterior] = compute_posterior(K, C, z_prior, y_measured)
% computes posterior values

  % calculate posterior augmented state
  z_posterior = z_prior + K * (y_measured - C * z_prior);

  % calculate covariance matrix
  KC = K * C;
  I = eye(size(KC));
  P_posterior = (I - KC) * P_prior;
end