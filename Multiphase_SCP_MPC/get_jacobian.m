function [A, B_minus, B_plus, S] = get_jacobian(x, u, u_, fsm_state, N_phase, dt, params)
% Calculates A and B Jacobian Matricies by linearising the dynamics model about x,u,u_
    % define parameters
    nx = length(x);
    nu = length(u);

    % Initialize continuous Jacobians
    Ac = zeros(nx, nx);
    Bc = zeros(nx, nu);
    
    % pertubation size
    epsilon = 1e-8;
    
    % baseline derivative
    f0 = dynamics_fn(x, ...
        u, u_, fsm_state, params);
    
    % Build dynamics matrix
    for i = 1 : nx
        x_pert = x;
        x_pert(i) = x_pert(i) + epsilon;
        f_pert = dynamics_fn(x_pert, ...
            u, u_, fsm_state, params);
        Ac(:, i) = (f_pert - f0) / epsilon;
    end
    
    % Build control matrix
    for i = 1 : nu
        u_pert = u;
        u_pert(i) = u_pert(i) + epsilon;
        f_pert = dynamics_fn(x, ...
            u_pert, u_, fsm_state, params);
        Bc(:, i) = (f_pert - f0) / epsilon;
    end 
    
    % Convert from continuous form to discrete FOH form
    M = [Ac,            Bc,            zeros(nx, nu) ;
         zeros(nu, nx), zeros(nu, nu), eye(nu, nu)   ;
         zeros(nu, nx), zeros(nu, nu), zeros(nu, nu) ];
    state_transition_mat = expm(M * dt);

    % Create FOH system
    % Extract block matrices
    F11 = state_transition_mat(1 : nx, 1 : nx);
    F12 = state_transition_mat(1 : nx, nx + 1 : nx + nu);
    F13 = state_transition_mat(1 : nx, nx + nu + 1 : nx + 2 * nu);

    % Create FOH system discrete matrices
    A = F11;
    B_plus = F13 * (1 / dt);
    B_minus = F12 - B_plus;

    % Compute time step sensitivity
    dx_ddt = f0 - (1 / dt) * B_plus * (u_ - u);
    S = dx_ddt * (1 / (N_phase - 1));
end