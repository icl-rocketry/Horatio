function [x_opt, u_opt] = MPC_loop(x_current, x_ref, u_ref, params)
    current_sol_x = x_ref;
    current_sol_x(:, 1) = x_current;
    current_sol_u = u_ref;

    for iter = 1:params.max_solver_iters
        [X_new, U_new, cvx_status] = SCP_step(current_sol_x, current_sol_u, x_current, params);

        if ~contains(cvx_status, 'solved')
            warning("Solver failed, initiating fallback");
            break;
        end

        delta_x = norm(X_new - current_sol_x);
        delta_u = norm(U_new - current_sol_u);
        total_change = delta_x + delta_u;

        current_sol_x = X_new;
        current_sol_u = U_new;

        if total_change < params.convergence_th
            disp("Solver Converged Early");
            break;
        end 
    end

    x_opt = current_sol_x;
    u_opt = current_sol_u;
end