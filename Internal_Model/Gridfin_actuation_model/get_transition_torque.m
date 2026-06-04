function [tau_trans] = get_transition_torque(state, system_config)
% Calculates the transition torque output by the servo due to backlash
    % Extract useful params
    Cg = system_config.gear_damping_coeff;
    k = system_config.torsional_gear_stiffness;
    N = system_config.gear_ratio;
    alpha = system_config.dead_zone;

    % define relative positions
    delta_theta = (state(1) / N) - state(3);
    delta_theta_dot = (state(2) / N) - state(4);

    % Calculate torque
    if delta_theta > alpha / 2
        tau_trans = k * (delta_theta - (alpha / 2)) + Cg * delta_theta_dot;
    elseif delta_theta < -alpha / 2
        tau_trans = k * (delta_theta + (alpha / 2)) + Cg * delta_theta_dot;
    else
        tau_trans = 0.0;
    end
end