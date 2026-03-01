function [x_opt, u_opt, sigma_opt, fsm_state] = MPC_loop(x_current, x_ref, u_ref, sigma_ref, fsm_state, telemetry, params)
% runs one MPC iteration to obtain optimal trajectory and FF control
    
    % State machine and horzion shrinking
    if fsm_state.current_phase == 1
        % If coast TTG drops below threshold trigger switch
        if sigma_ref(3) <= params.relight_trigger
            % switch phase
            fsm_state.current_phase = 2;
            
            % dump old phase nodes
            x_ref = x_ref(:, params.N_coast:end);
            u_ref = u_ref(:, params.N_coast:end);
            sigma_ref = sigma_ref(1:2); 
        end
        
    elseif fsm_state.current_phase == 2
        % If thrust spools past min threshold then trigger switch
        if telemetry.thrust >= params.spool_thrust_target
            % switch phase
            fsm_state.current_phase = 3;
            
            % dump old phase nodes
            x_ref = x_ref(:, params.N_relight:end);
            u_ref = u_ref(:, params.N_relight:end);
            sigma_ref = sigma_ref(1); 
        end
    end
    
    % spool end condition
    if fsm_state.current_phase == 2
        T_curr = telemetry.thrust;
        T_dot = telemetry.thrust_gradient;
        
        % Calculate remaining time until thrust target reached
        if T_dot > 0 
            t_rem = (params.spool_thrust_target - T_curr) / T_dot;
        else
            t_rem = params.predicted_relight_time; 
        end
        
        % Calculate node count for remaining time
        N_relight = ceil(t_rem / params.relight_dt) + 1;
        N_relight = max(2, min(N_relight, params.max_relight_nodes));
        
        % Interpolate if the required node count has shifted
        if N_relight ~= fsm_state.N_relight
            % create old and new grids
            grid_old = linspace(0, 1, fsm_state.N_relight);
            grid_new = linspace(0, 1, N_relight);
            
            % interpolate to get old solution onto new grid
            x_ref_relight = interp1(grid_old, x_ref(:, 1:fsm_state.N_relight)', grid_new, 'linear')';
            u_ref_relight = interp1(grid_old, u_ref(:, 1:fsm_state.N_relight)', grid_new, 'linear')';
            
            % Reconstruct the full arrays
            x_ref = [x_ref_relight, x_ref(:, fsm_state.N_relight+1:end)];
            u_ref = [u_ref_relight, u_ref(:, fsm_state.N_relight+1:end)];
            
            % change remaining node number
            fsm_state.N_relight = N_relight; 
            
            % place quaternions back onto manifold after regridding
            for k = 1:size(x_ref, 2)
                q_raw = x_ref(params.q_idx, k);
                x_ref(params.q_idx, k) = q_raw / norm(q_raw, 2);
            end
        end
        
        % anchor the time prediction for the solver
        sigma_ref(2) = t_rem; 
    end

    % create current soltuion but start it at current measured position
    current_sol_x = x_ref;
    current_sol_x(:, 1) = x_current;
    current_sol_u = u_ref;
    current_sol_sigma = sigma_ref;
    
    % iterate till trajectory converged or maximum iters reached
    for iter = 1:params.max_solver_iters
        % solve optimisation problem for current iteration
        [X_new, U_new, sigma_new, cvx_status] = SCP_step(current_sol_x, ...
            current_sol_u, current_sol_sigma, x_current, fsm_state, telemetry, params);
        
        % check if solve was successful, if not, dont update solution
        if ~contains(cvx_status, 'solved')
            break;
        end
        
        % check trajectory update distance
        delta_x = norm(X_new - current_sol_x);
        delta_u = norm(U_new - current_sol_u);
        total_change = delta_x + delta_u;
        
        % update solution
        current_sol_x = X_new;
        current_sol_u = U_new;
        current_sol_sigma = sigma_new;
        
        % if solver converged, break the loop
        if total_change < params.convergence_th
            break;
        end 
    end
    
    % output trajectory
    x_opt = current_sol_x;
    u_opt = current_sol_u;
    sigma_opt = current_sol_sigma;
end