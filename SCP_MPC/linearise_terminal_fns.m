function [L_h, grad_L, P_h, grad_P] = linearise_terminal_fns(x, u, x_ref, u_ref, params)
% linearises cost functions around reference trajectory

    % define state pertubation
    eps_state = params.eps_state;
    
    % define running cost across trajectory
    L_h = sum(x(:, 15));
    
    % define terminal cost at landing
    P_h = terminal_cost_fn(x(:, end));
    
    % for each 
    for k = 1:N
        grad_L = ((x + eps_state) - (x - eps_state)) / (2 * eps_state);
        grad_P = 0;
end