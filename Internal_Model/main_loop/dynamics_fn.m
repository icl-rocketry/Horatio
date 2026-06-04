function [x_dot, x_add_dot] = dynamics_fn(x, x_add, u, config)
% calculates force and torque on the body to get the state derivative of the system
    % Obtain Cg and inertia tensor
    [r_cg, J] = Get_inertia_and_mass(x(14), x_add(1), x_add(2), config);
    x_cg = r_cg(1);

    % Ensure quaternions are on the manifold
    q = x(7:10);
    x(7:10) = q / norm(q);
    
    % Initialise additonal state derivative vector
    x_add_dot = zeros(size(x_add));

    % Grid fin actuator model to get state derivative
    % x_add_GF = [theta_m, theta_m_dot, theta_L, theta_L_dot]
    for i = 1:3
        actuator_state = x_add((2 + i) : (2 + 4 * i));
        x_add_dot_GF = GF_actuator_dynamic_fn(actuator_state, ...
            u(i), config);
        x_add_dot((2 + 4 * (i - 1)) : (2 + 4 * i)) = x_add_dot_GF;
    end

    % Calculate aerodynamic force and torque in body axes
    [F_aero_body, tau_aero_body] = Aerodynamic_forces(x, ...
        u(1:3), x_cg, config);

    % Calculate engine gimbal rate
    % gimbal_state = [alpha_y, alpha_y_dot, alpha_z, alpha_z_dot]
    gimbal_state = x_add(15:18);
    [x_add_dot_gimbal] = Engine_gimbal_dynamics_fn(gimbal_state, ...
        u(4:5), config);
    
    % Calculate force and torque from engine in body axes
    [F_thrust_body, tau_thrust_body, x_add_dot_fuel] = Engine_Model(Isp, OF); % need to finish

    % Calculates state derivative with forces and torques
    x_dot = Get_state_derivative(x, ...
        J, F_thrust_body, tau_thrust_body, F_aero_body, ...
        tau_aero_body, config);
    
    % construct additional state derivative vector
    x_add_dot(1:2) = x_add_dot_fuel(:); 
    x_add_dot(3:6) = x_add_dot_gimbal(:);
end