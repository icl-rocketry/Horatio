function [x_guess, u_guess, sigma_guess] = cold_start(x_init, params)
% Generates the initial reference trajectory based on deployment state
    % extract parameters
    N_coast = params.N_coast;
    N_relight = params.N_relight_nominal;
    N_burn = params.N_burn;
    N = N_coast + N_relight + N_burn;
    idx_spool_start = N_coast + 1;
    idx_burn_start = N_coast + N_relight + 1;

    % instantiate cold output arrays
    u_guess = zeros(params.control_size, N);
    x_guess = zeros(params.state_size, N);
    x_guess(:, 1) = x_init;
    
    % calculate rough estimate phase times
    vz_nm_safe = min(x_init(6), -1.0);
    t_coast_guess = max(0.1, (x_init(3) - params.ignition_alt_guess) / abs(vz_nm_safe));
    t_relight_guess = params.predicted_relight_time;
    t_burn_guess = 10.0;
    sigma_guess = [t_burn_guess; t_relight_guess; t_coast_guess];
    
    % guess linear startup for spool (will change this to a model later)
    for k = idx_spool_start:(idx_burn_start - 1)
        fraction = (k - idx_spool_start) / (N_relight - 1);
        u_guess(6, k) = fraction * params.min_throttle;
    end
    
    % guess hover throttle for burn control input
    mass_current = x_init(14);
    hover_throttle = (mass_current * 9.81) / params.T_max;
    safe_hover = min(params.max_throttle, max(params.min_throttle, hover_throttle));
    u_guess(6, idx_burn_start:end) = safe_hover;
    
    % get target state
    x_target = params.target_state;
    
    % guess initial trajectory to be linear through state space to target
    for row = 1:params.state_size
        x_guess(row, :) = linspace(x_init(row), x_target(row), N);
    end
    
    % project quaternions back onto manifold
    for k = 1:N
        q_raw = x_guess(params.q_idx, k);
        x_guess(params.q_idx, k) = q_raw / norm(q_raw, 2);
    end
end