function [nu] = required_force_SMC(r_curr, v_curr, a_curr, q_curr, w_curr, w_dot_curr, r_ref, v_ref, q_ref, w_ref, w_dot_ref, params)
    
    % get translational errors
    err_r = r_curr - r_ref;
    err_v = v_curr - v_ref;
    err_a = a_curr - a_ref;
    
    % get position error surface
    sigma_pos = err_v + lambda_pos * err_r;
    sigma_pos_dot = err_a + lambda_pos * err_v;
    
    % get attitude errors
    conj_q_ref = q_ref;
    err_q = hamilton_product_SMC(conj_q_ref, q_curr);
    err_w = w_curr - w_ref;
    err_w_dot = w_dot_curr - w_dot_ref;
    err_q_dot = 0.5 * hamilton_product_SMC(err_q, [0; err_w]);

    % get attitude error surface
    scalar_err = err_q(1);
    vector_err = err_q(2:4);
    sigma_att = err_w + lambda_att * vector_err;
    sigma_att_pos = err_w_dot + lambda_att * err_q_dot;
    
    % concatenate error surfaces
    sigma = [sigma_pos; sigma_att];
    sigma_dot = [sigma_pos_dot; sigma_att_dot];

    nu = - R1 * tanh(sigma) - R2 * tanh(sigma_dot);
end