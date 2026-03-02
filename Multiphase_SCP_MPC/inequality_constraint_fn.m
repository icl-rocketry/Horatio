function [g] = inequality_constraint_fn(t, x, u, x_target, constraints, tightening_params)
% Hard constraint functions with tightening to prevent violation from relaxation
    % Extract current position
    r_current = x(1:3);
    
    % calculate relative distances to target
    dx = r_current(1) - x_target(1);
    dy = r_current(2) - x_target(2);
    dz = r_current(3) - x_target(3);
    
    % glideslope cone constraint
    g_cone = norm([dx, dy]) - dz * tan(constraints.gamma) + constraints.d_cone;
    
    % get velocity in body axes
    v_inertial = x(4:6);
    R_BI = quat2rotm(x(7:10));
    v_body = transpose(R_BI) * v_inertial;
    
    % define relative axis for AoA
    n_axis = [-1; 0; 0];
    
    % constrain glideslope and tighten
    g_aoa = norm(v_body) * cos(constraints.aoa_max) - dot(v_body, n_axis) + tightening_params.d5;
    
    % assemble the final column vector
    g = [g_cone; g_aoa];
end