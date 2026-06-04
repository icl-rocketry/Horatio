function [x_dot] = Get_state_derivative(x, J, F_thrust_body, tau_thrust_body, F_aero_body, tau_aero_body, config)
% Calculates state derivative vector given resultant forces, torques, mass and inertial moment
    % constants
    g0 = config.g0;

    % decompose state vector
    v = x(4:6);
    q = x(7:10);
    w = x(11:13);
    m = x(14);

    % get rotation matrix
    RBI = quat2rotm(q);
    
    % get total body force and torque
    F_body = F_thrust_body + F_aero_body; 
    tau_body = tau_thrust_body + tau_aero_body;

    % invert J and get gravitational vector
    inv_J = J ^ -1;
    g = [0; 0; -g0];
    
    % get state derivatives in inertial axes
    r_dot = v;
    v_dot = (1 / m) * (R_BI * F_body) + g;
    q_dot = Quaternion_derivative(q, w); % quaternion derivative taken in body reference frame. Note: this is equivilant to inertial frame w.o the need for transform.
    w_dot = inv_J * (tau_body - cross(w, J .* w));
    m_dot = - norm(F_thrust_body) / (Isp * g0); % check this

    m_dot_fuel = m_dot * (1 / (1 + OF));
    m_dot_ox = m_dot * (OF / (1 + OF));
    
    % create and assemble derivative vector
    x_dot = zeros(14, 1);
    x_dot(1:3) = r_dot;
    x_dot(4:6) = v_dot;
    x_dot(7:10) = q_dot;
    x_dot(11:13) = w_dot;
    x_dot(14) = m_dot;

    % create and assemble additional derivative vector for fuel compoents
    x_add_dot_prop = zeros(2, 1);
    x_add_dot_prop(1) = m_dot_fuel;
    x_add_dot_prop(2) = m_dot_ox;
end