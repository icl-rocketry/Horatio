function [h] = path_constraint_fn(t, x, u)
% Defines track for rocket and returns value if rocket passes off the track.
    % rocket should not follow a track therefore this value is always zero
    h = 0;
end

