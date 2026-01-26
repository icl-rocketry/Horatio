function [A, B] = get_jacobian(x, u, u_, epsilon_x, epsilon_u)
% Calculates A and B Jacobian Matricies by linearising the dynamics model about x,u.
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
        A(:, i) = (dynamics_fn(x + eps_x, u) - dynamics_fn(x - eps_x, u)) / (2 * eps_x(i));
    end 

    for i = 1:nu
        eps_u = zero_vec_u;
        eps_u_ = zero_vec_u;

        eps_u(i) = epsilon_u(i);
        eps_u_(i) = epsilon_u(i);
        B_minus(:, i) = (dynamics_fn(x, u + eps_u) - dynamics_fn(x, u - eps_u)) / (2 * eps_u(i));
        B_plus(:, i) = (dynamics_fn(x, u + eps_u_) - dynamics_fn(x, u - eps_u_)) / (2 * eps_u_(i));
    end 
end