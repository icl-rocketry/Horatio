function [x_new, u_new] = warm_start(x, u, params)
% Obtains next reference guess from shifting the phase of the previous unused steps and calculating the tail.
    N = params.N;
    K = params.K;
    dt = params.dt;

    % number of unused steps from previous solve
    overlap = N - K;

    % Phase shift
    x_new = zeros(size(x));
    u_new = zeros(size(u));
    x_new(:, 1:overlap) = x(:, K+1:end);
    u_new(:, 1:overlap) = u(:, K+1:end);

    % Extend control, N >> K for this to hold
    for k = overlap + 1 : N
        u_new(:, k) = u(:, end);
    end

    % Extend state
    current_x = x_new(:, num_shifted); 
    
    for k = overlap : N-1
        u_k = u_new(:, k); 
        u_k_ = u_new(:, k+1);
        next_x = dynamics_step(current_x, u_k, u_k_, dt, params);
        x_new(:, k+1) = next_x;
        current_x = next_x;
    end
end