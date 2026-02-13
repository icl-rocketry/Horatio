function [L_h, grad_L_h, P_h, grad_P_h] = linearise_terminal_fns(x_ref, params)
% linearises cost functions around reference trajectory

    % define state pertubation
    eps_state = params.eps_state;
    
    % define running cost across trajectory
    L_h = x_ref(15, end);
    
    % define terminal cost at landing
    W = params.terminal_weight;
    target_x = params.target_x;
    P_h = terminal_cost_fn(W, x_ref(:, end), target_x);
    
    % differentiate L and P around
    R = params.running_weight;
    pertubation = zeros(size(x_ref(:, end)));
    grad_L_h = zeros(nx, 1);
    grad_P_h = zeros(nx, 1);

    for k = 1:params.nx+2
        % create pertubated states
        pertubation(k) = eps_state;
        x_plus = x_ref(:, end) + eps_state;
        x_minus = x_ref(:, end) - eps_state;
        
        % snap quaternion back onto manifold after addition
        if ismember(k, params.q_idx)
            q_p = x_plus(params.q_idx);
            q_m = x_minus(params.q_idx);
            
            x_plus(params.q_idx)  = q_p / norm(q_p);
            x_minus(params.q_idx) = q_m / norm(q_m);
        end 
        
        % obtain derivatives of L and P wrt state pertubation
        grad_L_h(:, k) = (x_plus(k) - x_minus(k)) / (2 * eps_state);
        grad_P_h(:, k) = (terminal_cost_fn(W, x_plus(1:params.nx), target_x) - ...
            terminal_cost_fn(W, x_minus(1:params.nx), target_x)) / (2 * eps_state);
        
        % reset pertubation
        pertubation(k) = 0.0;
    end
end