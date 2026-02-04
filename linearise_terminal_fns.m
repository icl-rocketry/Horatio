function [L_h, grad_L, P_h, grad_P] = linearise_terminal_fns(x, u, x_ref, u_ref, params)
% linearises cost functions around reference trajectory
    eps_state = ;

    L_h = cost_fn();

    for k = 1:N
        grad_L = 0;

    P_h = terminal_cost_fn();
    grad_P = 0;
end