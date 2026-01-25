function [x_dot] = dynamics_fn(x, u)
    x_dot = [rx_dot, r_y_dot, rz_dot, vx_dot, vy_dot, vz_dot, qr_dot, qi_dot, qj_dot, qk_dot, wx_dot, wy_dot, wz_dot, m_dot];
end