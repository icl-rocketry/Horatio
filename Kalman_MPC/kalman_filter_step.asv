function [d_current, P_posterior] = kalman_filter_step(x_nav, x_prev, d_prev, P_prev, u_last, params)
% estimates disturbance term by statistical analysis of model deviation from measurement
    % form augmented state vector
    z_prev = [x_prev; d_prev];
    
    % linearise system at query point
    [A_aug, B_aug, C_aug] = discrete_linearise_system(x_nav, u_last, C, Bd, dt, params);
    
    % compute prior values from linearised model
    [z_prior, P_prior] = compute_prior(z_prev, u_last, P_prev, A_aug, B_aug, C_aug);
    
    % compute gain to weight error in posterior update
    K = compute_gain(C_aug, P_prior, R);
    
    % compute posterior values
    [z_posterior, P_posterior] = compute_posterior(K, C_aug, R, z_prior, P_prior, x_nav);
    
    % extract disturbance
    d_current = z_posterior(params.nx:end);
end