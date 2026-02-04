function [L] = cost_fn(R, u)
% Calculates the cost
    L = transpose(u) * R * u;
end