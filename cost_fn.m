function [cost] = cost_fn(R, u)
% Calculates the cost. 
    cost = transpose(u) * R * u; % Set to just minimise actuation
end