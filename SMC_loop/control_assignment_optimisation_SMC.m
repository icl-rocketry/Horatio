function [delta_u] = control_assignment_optimisation_SMC(nu, R, W, B)
% runs optimisation to convert required forces solved for by 2-SMC to corrective control signal, uses analytical optimisation solution.
    
    % define analytical optimisation matrices, weighted mean square cost optimisation
    mat_a = B' * W * nu;
    mat_b = B' * W * B + R;

    % solve for control signal
    delta_u = mat_b \ mat_a;
end