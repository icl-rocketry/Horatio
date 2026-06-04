function [A_aug, B_aug, C_aug] = discrete_linearise_system(x_measured, u_last, C, Bd, dt, params)
% Takes measured position + last input and linearises around to create linearised system
    % define parameters
    epsilon_x = params.eps_x;
    epsilon_u = params.eps_u;
    nx = params.nx;
    nu = params.nu;

    % create empty
    A_cont = zeros(nx, nx);
    B_cont = zeros(nx, nu);
    eps_x = zeros(nx, 1);
    eps_u = zeros(nu, 1);
    
    % continuous time linearisation of state
    for i = 1:nx
        % define perturbation vector
        eps_x(i) = epsilon_x;
        
        % create upper and lower perturbed states
        x_plus = x + eps_x;
        x_minus = x - eps_x;
        
        % snap quaternions back onto mainfold
        if ismember(i, params.q_idx)
            q_p = x_plus(params.q_idx);
            q_m = x_minus(params.q_idx);
            
            x_plus(params.q_idx)  = q_p / norm(q_p);
            x_minus(params.q_idx) = q_m / norm(q_m);
        end 
        
        % 2nd order FD to get gradients 
        A_cont(:, i) = (dynamics_step(x_plus, u_last, dt, params) - dynamics_step(x_minus, u_last, dt, params)) / (2 * epsilon_x);
        
        % reset pertubation vector
        eps_x(i) = 0;
    end 
    
    % continuous time linearisation of control input
    for i = 1:nu
        % define perturbation vector
        eps_u(i) = epsilon_u;
        
        % 2nd order FD to get gradients 
        B_cont(:, i) = (dynamics_step(x_meaured, u_last + eps_u, dt, params) - dynamics_step(x_measured, u - eps_u, dt, params)) / (2 * epsilon_u);
        
        % reset pertubation vector
        eps_u(i) = 0;
    end 
    
    % 2nd order taylor series to find discretised A
    A_disc = I + A_cont .* dt + 0.5 .* (A_cont * A_cont) .* (dt ^ 2);
    
    % 2nd order taylor series to find discretised B
    B_disc = (I .* dt + (A_cont .* (dt ^ 2) ./ 2)) * B_cont;

    % augment matrices
    A_aug = [A_disc, Bd; 0, I];
    B_aug = [B_disc; 0];
    C_aug = [C, 0];
end