function [F_aero_body, tau_aero_body] = Aerodynamic_forces(state, u_fins, x_cg, config)
% calculates aerodynamic forces and torques acting on the rocket in body axes
    
    % extract important values
    S_ref = config.S_ref; 
    L_ref = config.L_ref;
    x_gridfin = config.x_gridfin;
    S_ref_GF = config.S_ref_GF;
    R_rocket = config.R_rocket;
    chord_gridfins = config.chord_gridfins;
    Tables = config.Tables;

    % Calculate body to inertial axes rotation vector
    R_BI = quat2rotm(state(7:10));
    v_body = transpose(R_BI) * state(4:6);

    % calculate alpha and beta (z and y)
    alpha = atan2(v_body(3), -v_body(1));
    beta = atan2(v_body(2), -v_body(1));

    % obtain mach number and dynamic pressure
    flow_v = norm(v_body);
    Mach_n = get_mach(state(3), flow_v);
    [rho, ~, ~] = standard_atm_function(state(3));
    q = 0.5 * rho * (flow_v ^ 2);

    % calculate aerodynamic coefficients
    [C_X, C_Z, C_Y, C_l, C_m, C_n] = Aerodynamic_coeffs(state, ...
        v_body, flow_v, Mach_n, rho, L_ref, x_cg, S_ref, alpha, beta, ...
        u_fins, x_gridfin, S_ref_GF, R_rocket, chord_gridfins, Tables);

    % calculate aero forces
    Fx = C_X * q * S_ref;
    Fy = C_Y * q * S_ref;
    Fz = C_Z * q * S_ref;
    
    % calculate aero torques
    tau_x = C_l * q * S_ref * L_ref;
    tau_y = C_m * q * S_ref * L_ref;
    tau_z = C_n * q * S_ref * L_ref;
    
    % forculate force and torque vectors
    F_aero_body = [Fx, Fy, Fz];
    tau_aero_body = [tau_x, tau_y, tau_z];
end