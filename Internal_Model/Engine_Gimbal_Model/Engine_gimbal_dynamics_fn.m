function [actuator_state_dot] = Engine_gimbal_dynamics_fn(actuator_state, u, system_config) % act_state = [thetay, thetay_dot, thetaz, thetaz_dot]
% Gets rate of change of state from current state given an input signal for engine gimbal
    % Get accelaration rate of the system (30 deg/secondsquared for this)
    acceleration_rate = system_config.actuation_acceleration;

    % Get commanded gimble angle
    uy = u(4); 
    uz = u(5);

    % Get current gimble angle
    theta_y = actuator_state(1);
    theta_z = actuator_state(3);
    
    % System travels on linear path to target so acceleration vector points this way
    travel_unit_vec = zeros(2);
    travel_unit_vec(1) = uy - theta_y;
    travel_unit_vec(2) = uz - theta_z;
    acceleration_vector = acceleration_rate .* travel_unit_vec;

    % Get new actuation state derivative
    actuator_state_dot = zeros(4);
    actuator_state_dot(1) = actuator_state(2);
    actuator_state_dot(2) = acceleration_vector(1);
    actuator_state_dot(3) = actuator_state(4);
    actuator_state_dot(4) = acceleration_vector(2);
end