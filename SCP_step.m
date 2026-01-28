function [x_new, u_new] = SCP_step(x_ref, u_ref, x_current, params)
    N = params.N;

    % Linearisation
    for k = 1:N-1
        A(k), B_minus(k), B_plus(k) = get_jacobian(x_ref(k), u_ref(k), u_ref(k+1), params.eps_x, params.eps_u, dt, params);

        x_pred = A(k) * x_ref(k) + B_minus(k) * u_ref(k) + B_plus(k) * u_ref(k+1);
        x_real = dynamics_step(x_ref(k), u_ref(k), u_ref(k+1), dt, params);
        w(k) = x_real - x_pred;
    end 

    cvx_begin quiet

        % Variables
        variable X(params.state_size, N)
        variable U(params.control_size, N)
        variable mu_n(params.state_size, N-1)
        variable mu_p(params.state_size, N-1)
        variable nu_h(1)

        L, grad_L, P, grad_P = terminal_vals(X, U, params);

        J_cost = L + grad_L * (X(:, N) - x_ref(:, N)); 

        J_penalty = params.w_ep * nu_h + params.w_ep * sum(sum(mu_p + mu_m));
        J_prox_x = sum(sum_square(X - x_ref));
        J_prox_u = sum(sum_square(U - u_ref));
        J_prox = (params.w_prox / 2) * (J_prox_x + J_prox_u);
        
        minimize( J_cost + J_penalty + J_prox )
    
        % Enforce Constraints
        subject to
            X(:, 1) == x_current;
            
            for k = 1:N-1
                X(:, k+1) == A(k) * X(:, k) + B_minus(k) * U(:, k) + B_plus(k) * U(:, k+1) + w(k) + (mu_p(:, k) - mu_n(:, k)); 

                mu_n(:, k) >= 0;
                mu_p(:, k) >= 0;
           
                (X(end, k+1) - X(end, k)) <= params.epsilon;
            end

            for k = 1:N
                norm(U(:,k), 2) <= params.u_max;
            end

            P + grad_P * (X(:, N) - x_ref(:, N)) <= nu_h;

            nu_h >= 0;

    cvx_end
        
    % Update
    x_new = full(X);
    u_new = full(U);
end