function [U] = second_order_SMC(current_state, Uff, X_opt)
% Takes signal optimal signal Uff and adjusts it with 2-SMC corrective output to ensure it follows the optimal trajectory X_opt
    current_state_derivative = dynamics_step_SMC();

    r_curr = ;
    v_curr = ;
    a_curr = ;

    r_opt = ;
    v_opt = ;
    a_opt = ;

    q_curr = ;
    w_curr = ;
    w_dot_curr = ;

    q_opt = ;
    w_opt = ;
    w_dot_opt = ;

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
    B = get_control_authority_mat_SMC()

    delta_U = control_assignment_optimisation_SMC();
end

