function [F_thrust_body, tau_thrust_body, Isp] = Engine_dynamics_fn(Fmax, throttle, gimbal_angle_y, gimbal_angle_z, x_cg, x_engine)
% Obtains the thrust and torque due to the engine from the rockets inertial frame.
    % Obtain engine thrust output and define gimbled angle
    F_thrust = Engine_throttle_model(Fmax, throttle);
    total_gimble = sqrt((gimbal_angle_y ^ 2) + (gimbal_angle_z ^ 2));
    
    % Define force vecotrs from gimble angles
    Fx = F_thrust * sin(gimbal_angle_y);
    Fy = F_thrust * sin(gimbal_angle_z);
    Fz = F_thrust * cos(total_gimble);
    
    % Create force and torque vectors created by engine
    F_thrust_body = [Fx; Fy; Fz];
    tau_thrust_body = cross((x_engine - x_cg), F_thrust_body);
    Isp = 280; % Hard coded for now but make this a function
end