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
                if k < idx_coast_end
                    delta_sigma = sigma(3) - sigma_ref(3);
                elseif k >= idx_spool_start && k < idx_spool_end
                    delta_sigma = sigma(2) - sigma_ref(2);
                else
                    delta_sigma = sigma(1) - sigma_ref(1);
                end 
            elseif current_phase == 2
                if k < idx_spool_end
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
                         w(:, k) + ...
                         (mu_p(:, k) - mu_n(:, k)); 
            
            % constrain dynamic relaxation parameters
            mu_n(:, k) >= 0;
            mu_p(:, k) >= 0;
            
            % ensures that constraints are not violated by more than epsilon across step
            constraint_enforcement(X(end, k+1), X(end, k)) <= params.epsilon;
        end

        if current_phase == 1
            % coast constraints
            for k = idx_coast_start:idx_coast_end
                abs(U(1:3, k)) <= params.GF_max;
                U(4:6, k) == 0;
            end
            sigma(3) >= 0.1; 
            
            % relight constraints
            for k = idx_spool_start:idx_spool_end
                abs(U(1:3, k)) <= params.GF_max;
                U(4:5, k) == 0;
                U(6, k) == params.relight_throttle;
            end
            sigma(2) == params.predicted_relight_time; 
            
            % burn constraints
            for k = idx_burn_start:idx_burn_end
                U(6,k) <= params.min_throttle;
                U(6,k) <= params.max_throttle;
                abs(U(1:3,k)) <= params.GF_max;
                abs(U(4:5,k)) <= params.gimble_max;
            end
            sigma(1) >= 0.1;

        elseif curr_phase == 2
            % spool constraints - need to change this to make it adaptive
            for k = idx_spool_start:idx_spool_end
                abs(U(1:3, k)) <= params.GF_max;
                U(4:5, k) == 0;
                
                % Convert telemetry force to dimensionless throttle
                throttle_curr = telemetry.thrust / params.T_max;
                throttle_dot = telemetry.thrust_gradient / params.T_max;
                
                % Throttle command fed as current relight state
                % equivalent throttle (in reality throttle is held at a constant during spool up but this is to avoid crashing the solver)
                fraction = (k - 1) / (N_relight - 1);
                U(6, k) == throttle_curr + throttle_dot * (fraction * sigma(2));
            end

            % change to keep going
            sigma(2) >= params.min_spool_time; 
            sigma(2) <= params.max_spool_time; 
            
            % burn constraints
            for k = idx_burn_start:idx_burn_end
                U(6,k) >= params.min_throttle;
                U(6,k) <= params.max_throttle;
                abs(U(1:3,k)) <= params.GF_max;
                abs(U(4:5,k)) <= params.gimbal_max;
            end
            sigma(1) >= 0.1;

        elseif curr_phase == 3
            % burn constraints
            for k = idx_burn_start:idx_burn_end
                U(6,k) >= params.min_throttle;
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


% Vector sizing
nx = params.state_size + 2; % number of augmented states
nu = params.control_size;

% Indexing map
idx_X = 1 : (nx * N);
idx_U = idx_X(end) + 1 : idx_X(end) + (nu * N);
idx_mun = idx_U(end) + 1 : idx_U(end) + (nx * (N - 1));
idx_mup = idx_mun(end) + 1 : idx_mup(end) + (nx * (N - 1));
idx_nuh = idx_mup(end) + 1;
idx_sigma = idx_nuh + 1 : idx_nuh + num_active_phases;
idx_Jprox_x = idx_sigma(end) + 1;
idx_Jprox_u = idx_Jprox_x + 1;

% Initialise optimisation cost vector
nz = idx_Jprox_u;
c = zeros(nz, 1);

% linearise terminal cost and constraint
[~, grad_L, P, grad_P] = linearise_terminal_fns(xref, params);

% State cost linear setup, cx = [0_vec; grad_L; 1; 1]
c(idx_X(end - nx + 1 : end - 2)) = grad_L;
c(idx_X(end - 1)) = 1.0;
c(idx_X(end)) = 1.0;

% Penalty and weight setup (u cost terminal is zero)
c(idx_mun) = params.w_ep;
c(idx_mup) = params.w_ep;
c(idx_nuh) = params.w_ep;
c(idx_sigma) = params.w_time;
c(idx_Jprox_x) = params.w_prox / 2;
c(idx_Jprox_u) = params.w_prox / 2;

% Initialise sparse entry tracking
iA = zeros(); % row vector for all non zero entries in A
jA = zeros(); % column vector for all non zero entries in A
vA = zeros(); % vector storing the value of each sparse entry in A
nz_A = 0; % column tracker
row_eq = 0; % row tracker
b_eq = zeros(); % equality RHS vector

iG = zeros(); % row vector for all non zero entries in G
jG = zeros(); % column vector for all non zero entries in G
vG = zeros(); % vector storing the value of each sparse entry in G
h_ineq = zeros(); % inequality RHS vector
nz_G = 0; % column tracker
row_in = 0; % row tracker

% Equality constraints:

% Initial Conditions
num_val = nx;
row_eq_idx = (row_eq + 1 : row_eq + num_val);

% Store value and indexed positions
iA(nz_A + 1 : nz_A + num_val) = row_eq_idx;
jA(nz_A + 1 : nz_A + num_val) = idx_X(1 : num_val)';
vA(nz_A + 1 : nz_A + num_val) = 1;

% Setup RHS
b_eq(row_eq_idx, 1) = x_current(:); 

% Next state constraint:
for k = 1 : N-1
    % Get total time for current flight phase
    if current_phase == 1
        if k < idx_coast_end
            sig_idx = 3;
        elseif k >= idx_spool_start && k < idx_spool_end
            sig_idx = 2;
        else
            sig_idx = 1;
        end 
    elseif current_phase == 2
        if k < idx_spool_end
            sig_idx = 2;
        else
            sig_idx = 1;
        end 
    else
        sig_idx = 1;
    end
    
    % Rows of equality conditions needing to be filled, nx states constrained
    rows_k = (row_eq + 1 : row_eq + nx)';
    
    % Next state term X_k+1, all coefficient of 1
    iA(nz_A + 1 : nz_A + nx) = rows_k;
    jA(nz_A + 1 : nz_A + nx) = idx_X(k * nx + 1 : k * nx + nx)';
    vA(nz_A + 1 : nz_A + nx) = 1;
    nz_A = nz_A + nx;

    % Current state term X_k, coefficient of -A
    [r, c, v] = find(-A(:, :, k));
    num_val = length(v);
    if num_val > 0
        iA(nz_A + 1 : nz_A + num_val) = rows_eq + r;
        jA(nz_A + 1 : nz_A + num_val) = idx_X((k - 1) * nx + c)';
        vA(nz_A + 1 : nz_A + num_val) = val;
        nz_A = nz_A + num_val;
    end

    % Current control term U_k, coefficient of -B_minus
    [r, c, v] = find(-B_minus(:, :, k));
    num_val = length(v);
    if num_val > 0
        iA(nz_A + 1 : nz_A + num_val) = rows_eq + r;
        jA(nz_A + 1 : nz_A + num_val) = idx_U((k - 1) * nu + c)';
        vA(nz_A + 1 : nz_A + num_val) = val;
        nz_A = nz_A + num_val;
    end
    
    % Next control term U_k+1, coefficient of -B_plus
    [r, c, v] = find(-B_plus(:, :, k));
    num_val = length(v);
    if num_val > 0
        iA(nz_A + 1 : nz_A + num_val) = rows_eq + r;
        jA(nz_A + 1 : nz_A + num_val) = idx_U(k * nu + c)';
        vA(nz_A + 1 : nz_A + num_val) = val;
        nz_A = nz_A + num_val;
    end

    % Time dialation constraint sigma, coefficient of -S
    [r, c, v] = find(-S(:, :, k));
    num_val = length(v);
    if num_val > 0
        iA(nz_A + 1 : nz_A + num_val) = rows_eq + r;
        jA(nz_A + 1 : nz_A + num_val) = idx_sigma(sig_idx);
        vA(nz_A + 1 : nz_A + num_val) = val;
        nz_A = nz_A + num_val;
    end

    % slack contraint mu_p, coefficient of -1
    iA(nz_A + 1 : nz_A + nx) = rows_k;
    jA(nz_A + 1 : nz_A + nx) = idx_mup((k - 1) * nx + 1 : (k - 1) * nx + nx)';
    vA(nz_A + 1 : nz_A + nx) = -1;
    nz_A = nz_A + nx;

    % slack constraint mu_n, coefficient of 1
    iA(nz_A + 1 : nz_A + nx) = rows_k;
    jA(nz_A + 1 : nz_A + nx) = idx_mun((k - 1) * nx + 1 : (k - 1) * nx + nx)';
    vA(nz_A + 1 : nz_A + nx) = 1;
    nz_A = nz_A + nx;
    
    % setup RHS
    b_eq(rows_k, 1) = w(:, k) - S(:,:,k) * sigma_ref(sig_idx);
    
    % Advance row tracker
    row_eq = row_eq + nx;
end

% Quaternion constraint:
for k = 1 : N
    % extract reference quaternoin orientation
    q_bar = x_ref(params.q_idx, k);

    % create indexing
    row_eq = row_eq + 1;
    for i = 1 : length(params.q_idx)
        nz_A = nz_A + 1; 
        iA(nz_A) = row_eq; 
        jA(nz_A) = idx_X((k - 1) * nx + params.q_idx(i)); 
        vA(nz_A) = q_bar(i);
    end

    % constaint in RHS to keep Quaternion on manifold by maintaining magnitude 1
    b_eq(row_eq, 1) = 1;
end

% Coast and spool locks:
if current_phase == 1 % Currently coasting so solve through 3 phases
    % Get coast index list
    k_coast = (idx_coast_start : idx_coast_end);
    num_k = length(k_coast);

    % Coasting constraints
    if num_k > 0
        num_eq = 3 * num_k; % 3 controls locked per step
        rows = (row_eq + 1 : row_eq + num_eq)';
        
        % Broadcast indices: [4; 5; 6] + [0, nu, 2*nu...]
        rel_idx = (4:6)' + (k_coast - 1) * nu; 
        rel_idx = rel_idx(:); % Flatten to column vector
        
        iA(nz_A + 1 : nz_A + num_eq) = rows;
        jA(nz_A + 1 : nz_A + num_eq) = idx_U(rel_idx(:));
        vA(nz_A + 1 : nz_A + num_eq) = 1;
        b_eq(rows, 1) = 0;
        
        row_eq = row_eq + num_eq;
        nz_A   = nz_A + num_eq;
    end
    
    % Spool Constraints 
    k_spool = idx_spool_start : idx_spool_end;
    num_k = length(k_spool);
    if num_k > 0
        num_eq = 2 * num_k; % 2 controls locked per step
        rows = (row_eq + 1 : row_eq + num_eq)';
        
        rel_idx = (4:5)' + (k_spool - 1) * nu;
        
        iA(nz_A + 1 : nz_A + num_eq) = rows;
        jA(nz_A + 1 : nz_A + num_eq) = idx_U(rel_idx(:));
        vA(nz_A + 1 : nz_A + num_eq) = 1;
        b_eq(rows, 1) = 0;
        
        row_eq = row_eq + num_eq;
        nz_A   = nz_A + num_eq;
        
        % Spool Horizon: U(6) == relight_throttle
        num_eq = num_k; % 1 control locked per step
        rows = (row_eq + 1 : row_eq + num_eq)';
        
        rel_idx = 6 + (k_spool - 1) * nu;
        
        iA(nz_A + 1 : nz_A + num_eq) = rows;
        jA(nz_A + 1 : nz_A + num_eq) = idx_U(rel_idx(:));
        vA(nz_A + 1 : nz_A + num_eq) = 1;
        b_eq(rows, 1) = params.relight_throttle;
        
        row_eq = row_eq + num_eq;
        nz_A   = nz_A + num_eq;
    end
    
    % --- Single Equation: sigma(2) == predicted_relight_time ---
    row_eq = row_eq + 1;
    nz_A = nz_A + 1; 
    iA(nz_A) = row_eq; 
    jA(nz_A) = idx_sigma(2); 
    vA(nz_A) = 1; 
    b_eq(row_eq, 1) = params.predicted_relight_time;

elseif current_phase == 2 % Currently spooling so solve through 2 phases
    
    k_spool = idx_spool_start : idx_spool_end;
    num_k = length(k_spool);
    if num_k > 0
        % --- Spool Horizon: U(4:5) == 0 ---
        num_eq = 2 * num_k;
        rows = (row_eq + 1 : row_eq + num_eq)';
        rel_idx = (4:5)' + (k_spool - 1) * nu;
        
        iA(nz_A + 1 : nz_A + num_eq) = rows;
        jA(nz_A + 1 : nz_A + num_eq) = idx_U(rel_idx(:));
        vA(nz_A + 1 : nz_A + num_eq) = 1;
        b_eq(rows, 1) = 0;
        
        row_eq = row_eq + num_eq;
        nz_A   = nz_A + num_eq;
        
        % --- Adaptive Throttle Coupling: U_6 - (dot * frac)*sigma_2 = curr ---
        throttle_curr = telemetry.thrust / params.T_max;
        throttle_dot = telemetry.thrust_gradient / params.T_max;
        
        num_eq = num_k;
        rows = (row_eq + 1 : row_eq + num_eq)';
        
        % Indices and dynamic fractions
        rel_idx = 6 + (k_spool - 1) * nu;
        fractions = ((k_spool - 1) / (N_relight - 1))'; % Transposed to column
        
        % Term 1: 1 * U_6
        iA(nz_A + 1 : nz_A + num_eq) = rows;
        jA(nz_A + 1 : nz_A + num_eq) = idx_U(rel_idx(:));
        vA(nz_A + 1 : nz_A + num_eq) = 1;
        
        % Term 2: (-throttle_dot * fraction) * sigma(2)
        % (Assigned to the exact same rows as Term 1)
        iA(nz_A + num_eq + 1 : nz_A + 2*num_eq) = rows;
        jA(nz_A + num_eq + 1 : nz_A + 2*num_eq) = idx_sigma(2);
        vA(nz_A + num_eq + 1 : nz_A + 2*num_eq) = -throttle_dot * fractions;
        
        % Right-hand constant
        b_eq(rows, 1) = throttle_curr;
        
        row_eq = row_eq + num_eq;
        nz_A   = nz_A + 2 * num_eq; % Advanced by 2*num_eq because we added two terms
    end
end

% Inequality constraints: 

% Violation constraint:
% Number of equations added
num_eq = N - 1;

% Row indices for all equations
row_idx = (row_in + 1 : row_in + num_vals);

% Extract collision check index from augmented state vector for current and next state
idx_y_k = idx_X((1 : N - 1) * (nx + 2))';
idx_y_k_ = idx_X((2 : N) * (nx + 2))';

% Log positions in matrix for next state and give coefficient 1
iG(nz_G + 1 : nz_G + num_eq) = row_idx;
jG(nz_G + 1 : nz_G + num_eq) = idx_y_k_;
vG(nz_G + 1 : nz_G + num_eq) = 1;

% Log positions in matrix for current state and give coefficient -1
iG(nz_G + num_eq + 1 : nz_G + 2 * num_eq) = row_idx;
jG(nz_G + num_eq + 1 : nz_G + 2 * num_eq) = idx_y_k;
vG(nz_G + num_eq + 1 : nz_G + 2 * num_eq) = -1;

% Log RHS constraint
h_ineq(row_idx, 1) = params.epsilon;

% Move trackers forward
row_in = row_in + num_eq;
nz_G = nz_G + 2 * num_eq;

% Slack Limit Constraint: 
all_slacks = [idx_mun(:); idx_mup(:); idx_nuh];
num_slacks = length(all_slacks);
row_idx = (row_in + 1 : row_in + num_slacks)';

% Log slack positions and coefficients in G
iG(nz_G + 1 : nz_G + num_slacks) = row_idx;
jG(nz_G + 1 : nz_G + num_slacks) = all_slacks;
vG(nz_G + 1 : nz_G + num_slacks) = -1;

% Log RHS constraints
h_ineq(row_idx, 1) = 0;

% Move trackers forward
row_in = row_in + num_slacks;
nz_G = nz_G + num_slacks;

% Terminal Physical limits:
% grad_P * X_N - nu_h <= -P + grad_P * x_ref_N
% Step row down and get indexing
row_in = row_in + 1;
[idx_nz_P, ~, val_P] = find(grad_P(:)); 
num_nz = length(idx_nz_P);

% Setup inequality for terminal conditions
if num_nz > 0
    iG(nz_G + 1 : nz_G + num_nz) = row_in; 
    jG(nz_G + 1 : nz_G + num_nz) = idx_X(end - nx + idx_nz_P); 
    vG(nz_G + 1 : nz_G + num_nz) = val_P;
end

% Add nu condition
nz_G = nz_G + 1; 
iG(nz_G) = row_in; 
jG(nz_G) = idx_nuh; 
vG(nz_G) = -1;

% Setup RHS
h_ineq(row_in, 1) = - P + grad_P * x_ref(1 : nx_phys, N);

% Total time constraint:
row_in = row_in + 1;
for i = 1:num_active_phases
    nz_G = nz_G + 1; iG(nz_G) = row_in; jG(nz_G) = idx_sigma(i); vG(nz_G) = 1;
end
h_ineq(row_in, 1) = params.max_time;

% Lorentz cone dims

% Setup sparse matrix
A_sparse = sparse(iA, jA, vA, row_eq, nz);
G_sparse = sparse(iG, jG, vG, row_in, nz);

% Run ECOS solver to find optimal solution
[z_opt, ~, info] = ecos(c, G_sparse, h_ineq, dims, A_sparse, b_eq);