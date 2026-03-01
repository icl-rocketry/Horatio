function [x_new, u_new, sigma_new] = warm_start(prev_x, prev_u, prev_sigma, fsm_state, t_elapsed, params)
% Obtains next reference guess from shifting the phase of the previous unused steps and calculating the tail.
    
    % initialise new TTG and node count
    sigma_new = prev_sigma;
    N = size(prev_x, 2);

    % phase time deduction, remove solve time from 
    if fsm_state.current_phase == 1
        sigma_active_phase = prev_sigma(3);
        sigma_new(3) = max(0.1, sigma_new(3) - t_elapsed);
    elseif fsm_state.current_phase == 2
        sigma_active_phase = prev_sigma(2);
        sigma_new(2) = max(params.min_relight_time, sigma_new(2) - t_elapsed);
    else
        sigma_active_phase = prev_sigma(1);
        sigma_new(1) = max(0.1, sigma_new(1) - t_elapsed);
    end

    % define normalised change
    delta_tau = t_elapsed / sigma_active_phase;

    % create normalised time grids
    tau_old = linspace(0, 1, N);
    tau_new = linspace(delta_tau, 1 + delta_tau, N);

    % interpolate previous trajectory onto new grid
    x_shifted = interp1(tau_old, prev_x', tau_new, "linear", "extrap");
    u_shifted = interp1(tau_old, prev_u', tau_new, "linear", "extrap");
    
    % enforce boundary conditions on new extrapolated tail
    for k = 1:N
        if fsm_state.current_phase == 1
            if k < params.N_coast
                % gimble and throttle lock in coast
                u_shifted(4:6, k) = 0;
            elseif k >= params.N_coast && k < (params.N_coast + fsm_state.N_relight - 1)
                % gimble lock in relight
                u_shifted(4:5, k) = 0;
            end
        elseif fsm_state.current_phase == 2
            if k < fsm_state.N_relight
                % gimble lock in relight
                u_shifted(4:5, k) = 0;
            end
        end
    end
    
    % snap quaternions back onto manifold
    for k = 1:N
        q_raw = x_shifted(params.q_idx, k);
        x_shifted(params.q_idx, k) = q_raw / norm(q_raw, 2);
    end
    
    % create state and control outputs
    x_new = x_shifted;
    u_new = u_shifted;
    current_x = x_new(:, 1);
    
    % define internal node boundaries and precalculate phase specific dt values
    if fsm_state.current_phase == 1
        dt_coast = sigma_new(3) / (params.N_coast - 1);
        dt_spool = sigma_new(2) / (fsm_state.N_relight - 1);
        dt_burn  = sigma_new(1) / (params.N_burn - 1);
        idx_end_coast = params.N_coast;
        idx_end_spool = params.N_coast + fsm_state.N_relight - 1;
        
    elseif fsm_state.current_phase == 2
        dt_spool = sigma_new(2) / (fsm_state.N_relight - 1);
        dt_burn  = sigma_new(1) / (params.N_burn - 1);
        idx_end_spool = fsm_state.N_relight;
        
    elseif fsm_state.current_phase == 3
        dt_burn = sigma_new(1) / (params.N_burn - 1);
    end

    % Forward integration loop
    for k = 1:N-1
        % define timestep based on current phase
        if fsm_state.current_phase == 1
            if k < idx_end_coast
                dt_k = dt_coast; phase_idx = 1;
            elseif k >= idx_end_coast && k < idx_end_spool
                dt_k = dt_spool; phase_idx = 2;
            else
                dt_k = dt_burn; phase_idx = 3;
            end
       
        elseif fsm_state.current_phase == 2
            if k < idx_end_spool
                dt_k = dt_spool; phase_idx = 2;
            else
                dt_k = dt_burn; phase_idx = 3;
            end
            
        elseif fsm_state.current_phase == 3
            dt_k = dt_burn; phase_idx = 3;
        end
        
        % integrate dynamics forward
        x_new(:, k+1) = dynamics_step(current_x, u_new(:, k), u_new(:, k+1), dt_k, phase_idx, params);
        
        % Re-normalize attitude immediately after integration
        q_raw = x_new(params.q_idx, k+1);
        x_new(params.q_idx, k+1) = q_raw / norm(q_raw, 2);
        
        % step next state forwards
        current_x = x_new(:, k+1);
    end
end