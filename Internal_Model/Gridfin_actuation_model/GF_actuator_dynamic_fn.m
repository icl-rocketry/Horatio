function [actuator_state_dot] = GF_actuator_dynamic_fn(actuator_state, u, system_config)
% Obtains the rate of change of the gridfin state from current state
    % get torque output from the motor
    tau_m = get_motor_torque(actuator_state(1), ...
        actuator_state(2), u, system_config);

    % get transition torque from the gears
    tau_trans = get_transition_torque(actuator_state, system_config);

    % get hinge moment of the grid_fins
    % need to figure out how to handle this!
    tau_aero = get_GF_hinge_moment();
    
    % Calculate gridfin and motor acceleration
    theta_ddot_m = (1 / system_config.Im) * (tau_m - system_config.Cm * ...
        actuator_state(2) - (tau_trans / system_config.N));
    theta_ddot_L = (1 / system_config.IL) * (tau_trans - tau_aero - ...
        system_config.CL * actuator_state(4));

    % get state derivative vector
    actuator_state_dot = zeros(4);
    actuator_state_dot(1) = actuator_state(2);
    actuator_state_dot(2) = theta_ddot_m;
    actuator_state_dot(3) = actuator_state(4);
    actuator_state_dot(4) = theta_ddot_L;
end