function [U] = second_order_SMC(current_state, Uff, X_opt)
% Takes signal optimal signal Uff and adjusts it with 2-SMC corrective output to ensure it follows the optimal trajectory X_opt
    current_state_derivative = dynamics_step_SMC();
    X_opt_dot = dynamics_step_SMC();

    r_curr = current_state(1:3);
    v_curr = current_state(4:6);
    a_curr = current_state_derivative(4:6);

    r_opt = X_opt(1:3);
    v_opt = X_opt(4:6);
    a_opt = X_opt_dot(4:6);

    q_curr = current_state(7:10);
    w_curr = current_state(11:13);
    w_dot_curr = current_state_derivative(11:13);

    q_opt = X_opt(7:10);
    w_opt = X_opt(11:13);
    w_dot_opt = X_opt_dot(11:13);

    corrective_force = required_force_SMC();
    
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

    w_Fx = params.force_tracking_cost;
    w_Fy = params.force_tracking_cost;
    w_Fz = params.force_tracking_cost;
    w_taux = params.moments_tracking_cost;
    w_tauy = params.moments_tracking_cost;
    w_tauz = params.moments_tracking_cost;

    R = diag([r_fin_1, r_fin_2, r_fin_3, r_gimble_y, r_gimble_z]);
    W = diag([w_Fx, w_Fy, w_Fz, w_taux, w_tauy, w_tauz]);
    B = get_control_authority_mat_SMC();

    delta_U = control_assignment_optimisation_SMC();
end