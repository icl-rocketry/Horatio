function [B] = get_control_authority_mat_SMC(x_current, Uff, Uff_, dt, params)
% Linearises control response in dynamics model to obtain control authority matrix
    nx = params.state_size_SMC;
    nu = params.control_size_SMC;
    eps_u = params.epsilon_SMC_u;
    eps = zeros(nu);
    B = zeros(nx, nu);
    
    for k = 1:nu
        % create pertubation_vector for kth control signal
        eps(k) = eps_u;

        % get grad vector wit finite difference method
        grad_vec = (dynamics_step_SMC(x_current, Uff + eps, Uff_, dt, params) - dynamics_step_SMC(x_current, Uff - eps, Uff_, dt, params)) / (2 * eps);

        % snap quaternions back to manifold
        q = grad_vec(params.q_idx);
        q_snapped = q / norm(q);
        grad_vec(params.q_idx) = q_snapped;
        
        % set B terms as gradient
        B(1:3, k) = grad_vec(1:3);
        B(4:6, k) = grad_vec(4:6);
        B(7:11, k) = grad_vec(7:11);
        B(12:14, k) = grad_vec(12:14);

        % reset pertubation vector
        eps(k) = 0.0;
    end
end