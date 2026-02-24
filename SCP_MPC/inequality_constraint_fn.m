function [g] = inequality_constraint_fn(x, x1,x2,y1,y2, d1, d2, d3, d4, d5, aoa_max)
% Defines safety region for the rocket and will return a positive value if violated.

% passed in params:
% boundaries: [x1,x2,y1,y2], assume rectangle bound, axes aligned around landing site
% tightening parameters: [d1, d2, d3, d4, d5, aoa_max];

% current position: assuming position vector is relative to landing site
x_current = x(1);
y_current = x(2);

% find aoa
v_inertial = x(4:6);
R_BI = quat2rotm(x(7:10));
v_body = transpose(R_BI)*v_inertial;
aoa = acos( (dot(v_body, [-1,0,0])) / (norm(v_body)) ); % total angle of attack, always +ve

% evaluate g
g = [x_current - x2 + d1;
    x1 - x_current + d2;
    y_current - y2 + d3;
    y1 - y_current + d4;
    aoa - aoa_max + d5];

end

