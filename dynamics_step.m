function [x_] = dynamics_step(x, u, u_, dt, params)
% RK4 integration step with FOH control input
    u_grad = (u_ - u) / dt;
    k1 = dynamics_fn_aug(t, x, u, params);
    k2 = dynamics_fn_aug(t + (dt/2), x + (dt/2) * k1, u + u_grad * (dt/2), params);
    k3 = dynamics_fn_aug(t + (dt/2), x + (dt/2) * k2, u + u_grad * (dt/2), params);
    k4 = dynamics_fn_aug(t + dt, x + dt * k3, u_, params);
    x_ = x + dt * ((k1/6) + (k2/3) + (k3/3) + (k4/6));
end