function [z_prior, P_prior] = compute_prior(z_prev, u_prev, P_prev, A, B, Q)
% Computes prior predictions for state and covariance for comparison
    z_prior = A * z_prev + B * u_prev;
    P_prior = A * P_prev * transpose(P) + Q;
end

