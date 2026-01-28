function [x_guess, u_guess] = generate_initial_guess(x_init, params)
% Generates the initial reference trajectory based on deployment state.
    u_guess = zeros(params.control_size, params.N);
    x_guess = zeros(params.state_size, params.N);
    x_guess(:, 1) = x_init;

    for k = 1:params.N-1
        x_guess(:, k+1) = dynamics_step(x_guess(:, k), u_guess(:, k), u_guess(:, k+1), params.dt, params);
    end 
end