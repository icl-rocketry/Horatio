function [r_cg, J] = Get_inertia_and_mass(m_current, m_fuel, m_ox, config)
% Obtains the new mass the new centre of gravity and the new inertia tensor.
    % Constants
    r_dry = config.r_dry;
    m_dry = config.m_dry; 
    R_ox = config.R_ox;
    R_f = config.R_f;
    m_ox0 = config.m_ox0; 
    m_fuel0 = config.m_fuel0; 
    L0_ox = config.L0_ox; 
    L0_f = config.L0_f; 
    Base_ox = config.Base_ox; 
    Base_f = config.Base_f; 
    J_dry = config.J_dry;

    % get COM of rocket components
    L_f = L0_f * (m_fuel / m_fuel0);
    r_f = [Base_f + L_f; 0; 0];
    L_ox = L0_ox * (m_ox / m_ox0);
    r_ox = [Base_ox + L_ox; 0; 0];
    
    % get total cg position
    r_cg = (m_dry * r_dry + m_fuel * r_f + m_ox * r_ox) / (m_current);

    % get inertial components of rocket
    Ixx_f = 0.5 * m_fuel * (R_f ^ 2);
    Iyy_f = (1 / 12) * m_fuel * (3 * (R_f ^ 2) + (L_f ^ 2));
    Izz_f = Iyy_f;

    Ixx_ox = 0.5 * m_ox * (R_ox ^ 2);
    Iyy_ox = (1 / 12) * m_ox * (3 * (R_ox ^ 2) + (L_ox ^ 2));
    Izz_ox = Iyy_ox;

    J_f_loc = [Ixx_f, 0,     0;
               0,     Iyy_f, 0;
               0,     0,     Izz_f];

    J_ox_loc = [Ixx_ox, 0,      0;
                0,      Iyy_ox, 0;
                0,      0,      Izz_ox];

    % get transport matrices
    d_dry = r_dry - r_cg;
    d_f = r_f - r_cg;
    d_ox = r_ox - r_cg;

    T_dry = [d_dry(2)^2 + d_dry(3)^2, -d_dry(1)*d_dry(2), -d_dry(1)*d_dry(3);
        -d_dry(1)*d_dry(3), d_dry(1)^2 + d_dry(3)^2, -d_dry(1)*d_dry(2);
        -d_dry(1)*d_dry(3), -d_dry(1)*d_dry(2), d_dry(1)^2 + d_dry(2)^2];

    T_f = [d_f(2)^2 + d_f(3)^2, -d_f(1)*d_f(2), -d_f(1)*d_f(3);
        -d_f(1)*d_f(3), d_f(1)^2 + d_f(3)^2, -d_f(1)*d_f(2);
        -d_f(1)*d_f(3), -d_f(1)*d_f(2), d_f(1)^2 + d_f(2)^2];

    T_ox = [d_ox(2)^2 + d_ox(3)^2, -d_ox(1)*d_ox(2), -d_ox(1)*d_ox(3);
        -d_ox(1)*d_ox(3), d_ox(1)^2 + d_ox(3)^2, -d_ox(1)*d_ox(2);
        -d_ox(1)*d_ox(3), -d_ox(1)*d_ox(2), d_ox(1)^2 + d_ox(2)^2];

    % get inertial matrices wrt cg for each component
    J_dry_cg = J_dry + T_dry;
    J_f_cg = J_f_loc + T_f;
    J_ox_cg = J_ox_loc + T_ox;

    % get total inertial matrix
    J = J_dry_cg + J_f_cg + J_ox_cg;
end