function [x_new, u_new, sigma_new] = warm_start(prev_x, prev_u, sigma, params)
% Obtains next reference guess from shifting the phase of the previous unused steps and calculating the tail.
    N = params.N;
    K = params.K;

    old_dt = sigma / (N - 1);
    dt_elapsed = K * old_dt;

    % adjust sigma to be closer to target
    sigma_new = sigma - dt_elapsed;

    min_time = params.sigma_min;
    if sigma_new < min_time
        sigma_new = min_time;
    end 

    [X_new, U_new] = resample_trajectory(prev_x, prev_u, prev_sigma, sigma_new, dt_shift, params);

    current_x = X_new(:,1); 
    new_dt = sigma_new / (N - 1);
    for k = 1:N-1
       X_new(:, k+1) = dynamics_step(current_x, U_new(:,k), U_new(:,k+1), new_dt, params);
       current_x = X_new(:, k+1);
    end
end