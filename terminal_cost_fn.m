function [P] = terminal_cost_fn(W, x_terminal, target_landing)
% defines terminal cost function
    error = x_terminal - target_landing;
    P = transpose(error) * W * error;
end

