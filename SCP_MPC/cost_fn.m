function [L] = cost_fn(R, x, u, w_aoa, aoa_cost_trigger)
% Calculates the cost
    
    % actuator cost:
    L_actuatorcost = transpose(u) * R * u;
    
    % angle of attack cost:
    % find aoa
    v_inertial = x(4:6);
    R_BI = quat2rotm(x(7:10));
    v_body = transpose(R_BI)*v_inertial;
    aoa = acos( (dot(v_body, [-1,0,0])) / (norm(v_body)) ); % total angle of attack, always +ve
    da = aoa - aoa_cost_trigger;
    L_aoa = w_aoa * (da > 0) * da;

    % total cost
    L = L_actuatorcost + L_aoa;

end