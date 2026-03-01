function [x_new, u_new, sigma_new] = SCP_step(x_ref, u_ref, sigma_ref, x_current, fsm_state, telemetry, params)
% function to define the optimisation sub-problem and solve it for a single step
    
    % extract state machine information
    current_phase = fsm_state.current_phase;

    % allocate horizon length and dt dependent on phase
    if current_phase == 1
        % coast phase
        % calculate horizon length
        N_coast = params.N_coast;
        N_relight = fsm_state.N_relight;
        N_burn = params.N_burn;
        N = N_coast + N_relight + N_burn;

        % remaining phases
        num_active_phases = 3;
        
        % calculate timesteps for each phase
        dt_coast = sigma_ref(1) / (N_coast - 1);
        dt_relight = sigma_ref(2) / (N_relight - 1);
        dt_burn = sigma_ref(3) / (N_burn - 1);

    elseif current_phase == 2
        % relight phase
        % calculate horizon length
        N_relight = fsm_state.N_relight;
        N_burn = params.N_burn;
        N = N_relight + N_burn;

        % remaining phases
        num_active_phases = 2;
        
        % calculate timesteps for each phase
        dt_relight = sigma_ref(2) / (N_relight - 1);
        dt_burn = sigma_ref(3) / (N_burn - 1);

    else
        % burn phase
        % calculate horizon length
        N_burn = params.N_burn;
        N = N_burn;

        % remaining phases
        num_active_phases = 1;
        
        % calculate timesteps for each phase
        dt_burn = sigma_ref(3) / (N_burn - 1);
    end 
    
    % pre-allocate matrices
    A = zeros(params.state_size, params.state_size, N-1);
    B_minus = zeros(params.state_size, params.control_size, N-1);
    B_plus = zeros(params.state_size, params.control_size, N-1);
    S = zeros(params.state_size, 1, N-1);
    w = zeros(params.state_size, N-1);

    % Linearise dynamics around reference trajectory
    for k = 1:N-1
        % select dt based on current phase
        if current_phase == 1
            if k < N_coast
                dt_k = dt_coast; phase_idx = 1;
            elseif k >= N_coast && k < (N_coast + N_relight)
                dt_k = dt_relight; phase_idx = 2;
            else
                dt_k = dt_burn; phase_idx = 3;
            end 
        elseif current_phase == 2
            if k < N_relight
                dt_k = dt_relight; phase_idx = 2;
            else
                dt_k = dt_burn; phase_idx = 3;
            end 
        else
            dt_k = dt_burn; phase_idx = 3;
        end 
        
        % linearise around trajectory
        [A(:, :, k), B_minus(:, :, k), B_plus(:, :, k), S(:, :, k)] = get_jacobian(x_ref(k), ...
            u_ref(k), u_ref(k+1), phase_idx, params.eps_x, params.eps_u, params.eps_t, dt_k, params);
        
        % predict states with linearised from and dynamics model to obtain corrective factor
        x_pred = A(:, :, k) * x_ref(k) + B_minus(:, :, k) * u_ref(k) + B_plus(:, :, k) * u_ref(k+1);
        x_real = dynamics_step(x_ref(k), u_ref(k), u_ref(k+1), dt_k, phase_idx, params);
        w(:, k) = x_real - x_pred;
    end

    cvx_begin quiet

        % define optimiser variables
        variable X(params.state_size, N)
        variable U(params.control_size, N)
        variable mu_n(params.state_size, N-1)
        variable mu_p(params.state_size, N-1)
        variable nu_h(1)
        variable sigma(num_active_phases) % [sigma_burn, sigma_relight, sigma_coast]
        
        % linearise cost functions
        [L, grad_L, P, grad_P] = linearise_terminal_fns(xref, params);
        
        % obtain trajectory distance from refernce and normalise onto manifold
        d = X - x_ref;
        
        % obtain objective function
        J_cost = L + grad_L * d(:, end); 
        J_penalty = params.w_ep * nu_h + params.w_ep * sum(sum(mu_p + mu_n));
        J_prox_x = sum(sum_square(d));
        J_prox_u = sum(sum_square(U - u_ref));
        J_prox = (params.w_prox / 2) * (J_prox_x + J_prox_u);
        
        % define minimisation target
        minimize( J_cost + J_penalty + J_prox + params.w_time * sum(sigma) )
    
        % Enforce Constraints
        subject to
            
            % initial conditions
            X(:, 1) == x_current;
            
            for k = 1:N-1
                % get phase dependent sigma change
                if current_phase == 1
                    if k < N_coast
                        delta_sigma = sigma(3) - sigma_ref(3);
                    elseif k >= N_coast && k < (N_coast + N_relight)
                        delta_sigma = sigma(2) - sigma_ref(2);
                    else
                        delta_sigma = sigma(1) - sigma_ref(1);
                    end 
                elseif current_phase == 2
                    if k < N_relight
                        delta_sigma = sigma(2) - sigma_ref(2);
                    else
                        delta_sigma = sigma(1) - sigma_ref(1);
                    end 
                elseif current_phase == 3
                    delta_sigma = sigma(1) - sigma_ref(1);
                end

                % constrain dynamics to linearised form
                X(:, k+1) == A(:, :, k) * X(:, k) + ...
                             B_minus(:, :, k) * U(:, k) + ...
                             B_plus(:, :, k) * U(:, k+1) + ...
                             S(:, :, k) * delta_sigma + ...
                             w(k) + ...
                             (mu_p(:, k) - mu_n(:, k)); 
                
                % constrain dynamic relaxation parameters
                mu_n(:, k) >= 0;
                mu_p(:, k) >= 0;
                
                % ensures that constraints are not violated by more than epsilon across step
                constraint_enforcement(X(end, k+1), X(end, k)) <= params.epsilon;
            end

            if curr_phase == 1
                % coast constraints
                for k = 1:(N_coast - 1)
                    abs(U(1:3, k)) <= params.GF_max;
                    U(4:6, k) == zeros(3);
                end
                sigma(3) >= 0.1; 
                
                % relight constraints
                for k = idx_end_coast:(N_coast + N_relight - 2)
                    abs(U(1:3, k)) <= params.GF_max;
                    U(4:5, k) == zeros(2);
                    U(6, k) == params.relight_throttle;
                end
                sigma(2) == params.predicted_relight_time; 
                
                % burn constraints
                for k = (N_coast + N_relight - 2):N
                    U(6,k) <= params.min_throttle;
                    U(6,k) <= params.max_throttle;
                    abs(U(1:3,k)) <= params.GF_max;
                    abs(U(4:5,k)) <= params.gimble_max;
                end
                sigma(1) >= 0.1;

            elseif curr_phase == 2
                % spool constraints - need to change this to make it adaptive
                for k = 1:(N_relight - 1)
                    abs(U(1:3, k)) <= params.GF_max;
                    U(4:5, k) == 0;
                    
                    % Convert telemetry force to dimensionless throttle
                    throttle_curr = telemetry.thrust / params.T_max;
                    throttle_dot = telemetry.thrust_gradient / params.T_max;
                    
                    % Throttle command fed as current relight state
                    % equivilant throttle (in reality throttle is held at a constant during spool up but this is to avoid crashing the solver)
                    fraction = (k - 1) / (N_relight - 1);
                    U(6, k) == throttle_curr + throttle_dot * (fraction * sigma(2));
                end

                % change to keep going
                sigma(2) >= params.min_spool_time; 
                sigma(2) <= params.max_spool_time; 
                
                % burn constraints
                for k = N_relight:N
                    U(6,k) <= params.min_throttle;
                    U(6,k) <= params.max_throttle;
                    abs(U(1:3,k)) <= params.GF_max;
                    abs(U(4:5,k)) <= params.gimbal_max;
                end
                sigma(1) >= 0.1;

            elseif curr_phase == 3
                % burn constraints
                for k = 1:N
                    U(6,k) <= params.min_throttle;
                    U(6,k) <= params.max_throttle;
                    abs(U(1:3,k)) <= params.GF_max;
                    abs(U(4:5,k)) <= params.gimbal_max;
                end
                sigma(1) >= 0.1;
            end

            for k = 1:N
                % ensure that quaternions
                q_bar = x_ref(params.q_idx, k);
                q_bar' * X(params.q_idx, k) == 1;
            end

            % enforce terminal cost must be less than relaxation limit
            P + grad_P * (X(:, N) - x_ref(:, N)) <= nu_h;
            
            % relaxation limit must be positive
            nu_h >= 0;

            % limit total horizon time to be no further than t_max
            sum(sigma) <= params.max_time;

    cvx_end
        
    % Update optimal trajectory, control signal and horizon time
    x_new = full(X);
    u_new = full(U);
    sigma_new = sigma;
    
    % ensure quaternions are on manifold 
    for k = 1:N
        q_raw = x_new(params.q_idx, k);
        x_new(params.q_idx, k) = q_raw / norm(q_raw);
    end
end