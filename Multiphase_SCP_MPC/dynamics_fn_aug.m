function [x_dot] = dynamics_fn_aug(t, state, u, fsm_state, params)
% augments the state vector to include the cost and violation measure
    
    % get state derivative and phase
    phase_idx = fsm_state.current_phase;
    state_dot = dynamics_fn(t, state, u, phase_idx, params);

    % get current cost derivative
    l_dot = cost_fn(params.R, state, u);
    
    % calculate state and control violation from defined constraints
    g_vec = inequality_constraint_fn(t, ...
        state, u, params.x_target, params.violation_constraints, ...
        params.tightening_params);
    g_violation = sum(max(0, g_vec) .^ 2);
    h_vec = path_constraint_fn(t, state, u, fsm_state);
    h_violation = sum(h_vec .^ 2);
    
    % Calculate violation "Energy"
    y_dot = g_violation + h_violation;
    
    % augement state derivative vector
    x_dot = [state_dot; l_dot; y_dot];
end