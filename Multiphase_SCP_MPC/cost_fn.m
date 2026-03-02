function [L] = cost_fn(R, x, u)
% Calculates the cost of a state and action
    % punish actuator effort
    L = transpose(u) * R * u;
end