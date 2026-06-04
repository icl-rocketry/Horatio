function [tau_m] = get_motor_torque(theta_m, theta_dot_m, theta_cmd, system_config)
% Calculates the motor output torque
    % predict internal PD command
    e = theta_cmd - theta_m;
    Vcmd = system_config.Kp * e - system_config.Kd * theta_dot_m;
    Vcmd = max(-system_config.Vmax, min(system_config.Vmax, Vcmd));

    % calculate motor torque
    coeff = system_config.tau_stall / system_config.omega_no_load;
    tau_m = coeff * Vcmd - coeff * theta_dot_m;
    tau_m = max(-system_config.tau_max, min(system_config.tau_max, tau_m));
end