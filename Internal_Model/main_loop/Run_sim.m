function [y_array, t_array] = Run_sim(y_I, t_I, config)
rocket_config = struct("mass", 100.0, "L_ref", 5.0, "R_ref", 0.1, "x_cg", -2.0, "isp", 280, "T_max", 1000.0);
[aero_trans_table, aero_rot_table, thrust_table] = LoadAeroTables();
y = y_I; t = t_I;
steps = (config.T - t_I) / config.dt;
y_array = zeros(steps, 14);
t_array = zeros(steps);
y_array(0,:) = y; t_array(0) = t_I;
while (t < config.T && y(3) >= 0)
    control_signal = controller(y);
    y_ = RK4_step(t, y, config.dt, control_signal);
    t = t + dt;

    y_array(step, :) = y_;
    t_array(step) = t;
    y = y_;
end

plot3(y_array(1), y_array(2), y_array(3));
end