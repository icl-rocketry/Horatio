function [u] = PID(target, current, Kp, Ki, Kd)
    err = current - target;
    u = Kp * err + Ki * + Kd
end