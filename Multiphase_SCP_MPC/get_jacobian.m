function [A, B_minus, B_plus, S] = get_jacobian(x, u, u_, fsm_state, N_phase, dt, params)
% Calculates A and B Jacobian Matricies by linearising the dynamics model about x,u,u_
    % define parameters
    nx = length(x);
    nu = length(u);
    
    % Build dynamics matrix
    for i = 1 : nx
        Ac(:, i) = dynamics_fn(x, ...
            u, u_, fsm_state, params);
    end
    
    % Build control matrix
    for i = 1 : nu
        Bc(:, i) = dynamics_fn(x, ...
            u, u_, fsm_state, params);
    end 
    
    % Convert from continuous form to discrete FOH form
    M = [Ac, Bc, 0    ;
         0 , 0 , eye(nu, nu);
         0 , 0 , 0    ];
    state_transition_mat = expm(M);

    % Create FOH system
    A = state_transition_mat(1, 1);
    B_plus = state_transition_mat(1, 3) * (1 / dt);
    B_minus = state_transition_mat(1, 2) - B_plus;

    % Compute time step sensitivity
    vel_next = dynamics_fn(x, ...
            u, u_, fsm_state, params);
    dx_ddt = vel_next - (1 / dt) * B_plus * (u_ - u);
    S = dx_ddt * (1 / (N_phase - 1));
end