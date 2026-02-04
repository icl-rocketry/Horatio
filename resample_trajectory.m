function [X_new, U_new] = resample_trajectory(prev_x, prev_u, prev_sigma, new_sigma, dt_shift, params)
% maps previous temporal grid to new temporal grid to accomodate change in timestep

    % horizon length
    N = params.N;
    
    % define previous grid
    prev_dt = prev_sigma / (N - 1);
    prev_grid = linspace(0, prev_sigma, N);
    
    % define new grid
    t_new_start = dt_shift; 
    t_new_end = dt_shift + new_sigma;
    current_grid = linspace(t_new_start, t_new_end, N);
    
    % interpolate between states
    X_new = interp1(prev_grid, prev_x, current_grid, 'linear', 'extrap')';
    
    % interpolate between controls (FOH control so linear interp is fine)
    U_new = interp1(prev_grid, prev_u, current_grid, 'linear', 'extrap')';
    
    % Project quaternions back onto the manifold
    idx = params.q_idx;
    for k = 1:N
        q_raw = X_new(idx, k);
        X_new(idx, k) = q_raw / norm(q_raw);
    end
end