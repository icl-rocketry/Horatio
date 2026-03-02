function [E] = constraint_enforcement(x_, x)
% Enforces g and h boundary constraints across a trajectory step
    % check the difference between the "violation energy" doesnt increase between steps
    E = x_(16) - x(16);
end