function [x_dot] = dynamics_fn_aug(t, state, u, params)
% augments the state vector to include the cost and violation measure.
    state_dot = dynamics_fn(t, state, u, params);
    l_dot = cost_fn(params.R, u);
    
    g_violation = max(0, inequality_constraint_fn()) ^ 2;
    h_violation = (equality_constraint_fn()) ^ 2;

    y_dot = g_violation + h_violation;

    x_dot = [state_dot, l_dot, y_dot];
end

