function [delta_U] = second_order_SMC(current_state, Uff, X_opt, sigma, t, t_solve, dt, params)
% Takes signal optimal signal Uff and adjusts it with 2-SMC corrective output to ensure it follows the optimal trajectory X_opt
    
    % Interpolate to get the state and control signal at t and t_
    dt_MPC = sigma / (params.N - 1);

    k = floor((t - t_start) / dt_MPC);
    T_k = t_solve + k * dt_MPC;
    alpha = (t - T_k) / dt_MPC;

    Uff_t = (1 - alpha) * Uff(:, k) + alpha * Uff(:, k + 1);
    X_opt_t = (1 - alpha) * X_opt(:, k) + alpha * X_opt(:, k + 1);
    
    t_ = t + dt;
    k_ = floor((t_ - t_start) / dt_MPC);
    T_k_ = t_solve + k_ * dt_MPC;
    alpha_ = (t_ - T_k_) / dt_MPC;

    Uff_t_ = (1 - alpha_) * Uff(:, k_) + alpha_ * Uff(:, k_ + 1);

    % calculate state derivative for current state and optimal state at t
    current_state_derivative = dynamics_step_SMC(current_state, Uff_t, Uff_t_, dt, params);
    X_opt_dot_t = dynamics_step_SMC(X_opt_t, Uff_t, Uff_t_, dt, params);
    
    % decompose surrent and optimal state into components
    r_curr = current_state(1:3);
    v_curr = current_state(4:6);
    a_curr = current_state_derivative(4:6);

    r_opt = X_opt_t(1:3);
    v_opt = X_opt_t(4:6);
    a_opt = X_opt_dot_t(4:6);

    q_curr = current_state(7:10);
    w_curr = current_state(11:13);
    w_dot_curr = current_state_derivative(11:13);

    q_opt = X_opt_t(7:10);
    w_opt = X_opt_t(11:13);
    w_dot_opt = X_opt_dot_t(11:13);

    % calculate required forces and torques to return to optimal trajectory
    corrective_force = required_force_SMC(r_curr, v_curr, a_curr, q_curr, ...
        w_curr, w_dot_curr, r_opt, v_opt, a_opt, q_opt, w_opt, w_dot_opt, C1, C2);
    
    % define actuator cost matrix dependent on if engine is lit
    if is_lit == true
        r_fin_1 = params.cost_GF_El;
        r_fin_2 = params.cost_GF_El;
        r_fin_3 = params.cost_GF_El;
        r_gimble_y = params.gimble_El;
        r_gimble_z = params.gimble_El;
    else 
        r_fin_1 = params.cost_GF_Enl;
        r_fin_2 = params.cost_GF_Enl;
        r_fin_3 = params.cost_GF_Enl;
        r_gimble_y = params.gimble_Enl;
        r_gimble_z = params.gimble_Enl;
    end 
    
    % define tracking cost matrix
    w_Fx = params.force_tracking_cost;
    w_Fy = params.force_tracking_cost;
    w_Fz = params.force_tracking_cost;
    w_taux = params.moments_tracking_cost;
    w_tauy = params.moments_tracking_cost;
    w_tauz = params.moments_tracking_cost;
    
    % build matricies and calculate B
    R = diag([r_fin_1, r_fin_2, r_fin_3, r_gimble_y, r_gimble_z]);
    W = diag([w_Fx, w_Fy, w_Fz, w_taux, w_tauy, w_tauz]);
    B = get_control_authority_mat_SMC(current_state, Uff_t, Uff_t_, dt, params);
    
    % run optimisation to obtain control signal correction 
    delta_U = control_assignment_optimisation_SMC(corrective_force, R, W, B);
end