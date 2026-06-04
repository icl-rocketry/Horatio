function [T] = relight_thrust_model(T_relight_obj, t_relight, t)
% calculates the output thrust of the engine during relight phase
    % For now just a linear fit in time
    T = T_relight_obj * (t / t_relight);
end