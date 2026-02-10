function [K] = compute_gain(C, P_prior, R)
% Computes gain matrix that minimises MSE between observation and prior to compute posterior
  % compute submatrices
  C_T = transpose(C);
  S = C * P_prior * C_T;
  M = P_prior * C_T; 

  % solve for gain matrix
  K = S \ M;
end

