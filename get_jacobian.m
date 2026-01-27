function [A, B_minus, B_plus] = get_jacobian(x, u, u_, epsilon_x, epsilon_u, dt, params)
% Calculates A and B Jacobian Matricies by linearising the dynamics model about x,u,u_.
    nx = length(x);
    nu = length(u);
    A = zeros(nx, nx);
    B_minus = zeros(nx, nu);
    B_plus = zeros(nx, nu);

    zero_vec_x = zeros(nx, 1);
    zero_vec_u = zeros(nu, 1);
    
    for i = 1:nx
        eps_x = zero_vec_x;
        eps_x(i) = epsilon_x(i);
        A(:, i) = (dynamics_step(x + eps_x, u, u_, dt, params) - dynamics_step(x - eps_x, u, u_, dt, params)) / (2 * eps_x(i));
    end 

    for i = 1:nu
        eps_u = zero_vec_u;
        eps_u_ = zero_vec_u;

        eps_u(i) = epsilon_u(i);
        eps_u_(i) = epsilon_u(i);
        B_minus(:, i) = (dynamics_step(x, u + eps_u, u_, dt, params) - dynamics_step(x, u - eps_u, u_, dt, params)) / (2 * eps_u(i));
        B_plus(:, i) = (dynamics_step(x, u, u + eps_u_, dt, params) - dynamics_step(x, u, u - eps_u_, dt, params)) / (2 * eps_u_(i));
    end 
end