function [L] = cost_fn(R, x, u)
% Calculates the cost
    L = transpose(u) * R * u;
end